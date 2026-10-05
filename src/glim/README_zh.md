# GLIM（Milla fork）：长距离建图的 GPU 显存卸载

[English](README.md#milla-fork-gpu-memory-offloading-for-long-term-mapping)

## 背景

使用 `VGICP_GPU` 时，每个 submap 的点云和 voxelmap 都常驻在 GPU 上，按 Milla 的配置每个约 **7 MB**。显存占用随建图时间线性增长：20 分钟左右的路线约 6 GB，一小时以上的路线在 8–12 GB 的显卡上会溢出。

开启 GPU 卸载后，全局建图模块会把**离当前位置较远的 submap** 的 GPU 数据卸载到内存（RAM）。车辆重新靠近、这些 submap 可能参与回环检测时，再提前加载回 GPU。

## 前置条件

这个功能依赖 **gtsam_points** 里的一处修复（分支 `feature/dynamicLoardGP`，`IntegratedVGICPDerivatives::touch_points()`）。

没有这个修复时，多个因子共享同一份已卸载的数据，会在不同的 CUDA stream 上并发地重新加载它，导致**算出错误的 Hessian**，或者程序崩溃并报 `cudaErrorIllegalAddress`。开启功能前，必须先从这个分支编译并安装 gtsam_points：

```bash
cd gtsam_points && rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=89
make -j$(nproc) && sudo make install && sudo ldconfig
```

检查是否已经装上新库（时间戳应该是你安装的时间）：

```bash
ls -l --time-style=long-iso /usr/local/lib/libgtsam_points_cuda.so.1.2.1
```

## 工作原理

每插入一个 submap，按它到当前位置的距离做一次调度：

```
0 ─────── 150 m ─────────────── 250 m ──────→ 到当前位置的距离
  预加载区     │    缓冲区（保持原状）   │   卸载区
（确保在 GPU）                         （卸载到 RAM）
```

- **预加载半径** = `max_implicit_loop_distance`（100 m）+ `gpu_offload_prefetch_margin`（50 m）= 150 m
- **卸载距离** = `gpu_offload_distance`（250 m）
- **最新的 N 个 submap**（`gpu_offload_keep_recent_submaps`）不管距离多远，都始终留在 GPU 上
- 距离规则执行完后，如果 GPU 上的 submap 数据还超过 `gpu_offload_max_resident_mb`，就在可以卸载的 submap 里按**最久没被访问**的顺序继续卸载
- 已卸载的 submap 在 iSAM2 里被冻结（`noRelinKeys`），沿用缓存的线性化结果，不会被重新加载

## 参数说明（`config/config_global_mapping_gpu.json`）

| 参数 | 代码默认值 | Milla 配置值 | 建议范围 |
|---|---|---|---|
| `enable_gpu_offload` | `false` | `true` | — |
| `gpu_offload_keep_recent_submaps` | `30` | `80` | 15–100 |
| `gpu_offload_prefetch_margin` | `50.0` | `50.0` | 20–100 m |
| `gpu_offload_distance` | `250.0` | `250.0` | 预加载半径 + 50 m 以上 |
| `gpu_offload_max_resident_mb` | `0.0`（不限制） | `4000.0` | 见下方显卡对照表 |
| `gpu_offload_freeze_relinearization` | `true` | `true` | 保持 `true` |

代码默认值保持上游 GLIM 的行为（不卸载）。

### `enable_gpu_offload`

是否开启 GPU 卸载，只在 `registration_error_factor_type: "VGICP_GPU"` 时生效。

- **需要开启**：建图时间超过 20–30 分钟，或者显卡显存 ≤ 12 GB
- **可以关闭**：短距离建图（显存本来就够），或者需要和原版逐位对比结果时

### `gpu_offload_keep_recent_submaps`

最新的 N 个 submap 始终留在 GPU 上，不会因为距离或预算被卸载。

**为什么需要**：最近的 submap 还没收敛，iSAM2 会频繁地重新线性化它们，相邻 submap 之间也有匹配因子。如果它们被卸载，就会被冻结，这一段就没法继续优化了；而且每次更新都会被重新加载回来。车速快时，最近的几十个 submap 可能已经超出 250 m，这个参数按时间先后兜底，不依赖距离。

**怎么估算**：按这次的实测数据，平均每 **1.5 s** 生成一个 submap，所以 N 个 submap 约等于最近 1.5 × N 秒的轨迹。每个约 7 MB，80 个约 560 MB。

| 场景 | 建议值 |
|---|---|
| 城市低速、GNSS 稳定 | 30–50 |
| 高速或城际，或者 GNSS 修正频繁、幅度大 | 50–100 |
| 显存很紧张（8 GB） | 15–30，不建议低于 10 |
| 出现 `no more submaps can be offloaded` 警告 | 调小，或者调大 `gpu_offload_max_resident_mb` |

### `gpu_offload_prefetch_margin`

在回环搜索半径（`max_implicit_loop_distance`）之外多留的预加载缓冲距离，单位是米。

**为什么需要**：回环检测只在 100 m 内找候选。多留一段，是为了在候选进入 100 m 之前就把它们加载好，同时也能容忍回环或 GNSS 修正后 submap 位置的跳动。预加载没覆盖到时结果也不会出错，只是检测回环时要临时加载，会多停顿一下。

| 场景 | 建议值 |
|---|---|
| 一般情况 | 50 m |
| 车速快，或者 GNSS 修正幅度大 | 80–100 m |
| 显存紧张 | 20–30 m（临时加载的次数会变多） |
| 不建议 | 0（候选一进入 100 m 就得马上加载） |

### `gpu_offload_distance`

离当前位置超过这个距离的 submap 会被卸载，单位是米。

**为什么需要**：它和预加载半径之间的这段是缓冲区，缓冲区里的 submap 保持原状，避免车在边界附近来回开时反复加载和卸载。

- 必须**大于预加载半径**（默认 150 m）。设得更小时，代码会自动取预加载半径。
- 建议至少比预加载半径大 50 m，比如 200–300 m。
- 在一片区域里反复绕圈（园区、停车场）时，可以调大到 400–500 m，减少反复卸载和加载。
- 显存紧张时可以调小到 200 m，但缓冲区越窄，越容易反复切换。

### `gpu_offload_max_resident_mb`

允许 submap 数据在 GPU 上常驻的总量上限，单位 MB。`<= 0` 表示不限制，只按距离卸载。

**为什么需要**：距离规则管不到一种情况：在一片区域里反复绕圈时，大量 submap 都在 250 m 以内，一个也不会被卸载。预算规则用来兜底：超出上限后，按最久没被访问的顺序继续卸载。最新的 N 个和预加载区里的 submap 不会因为预算被卸载。

**统计范围**：只统计全局建图里 submap 的点云和 voxelmap，**不包括**里程计、子图构建（`sub_mapping.enable_optimization: true` 时会占更多）和 viewer 的显存。设置时要给这些模块留出余量，至少 2–3 GB。

| 显卡显存 | 建议值 |
|---|---|
| 8 GB | 2000–3000 |
| 12 GB | 4000–5000 |
| 16 GB 以上 | 6000 以上 |

按每个 submap 约 7 MB 算，4000 MB 大约能放 570 个 submap，远多于"最新 N 个 + 150 m 内"通常需要的数量。

### `gpu_offload_freeze_relinearization`

已卸载的 submap 不参与重新线性化，iSAM2 直接使用缓存的线性化结果。

- **保持 `true`**。设成 `false` 时，已卸载的 submap 在每次重新线性化时都会被加载回来，卸载基本不起作用。
- **代价**：远处的区域在回环修正后不会重新线性化，精度可能略有下降。GNSS 先验照样生效，影响有限。

## 按场景推荐的配置

| 场景 | `keep_recent` | `prefetch_margin` | `distance` | `max_resident_mb`（12 GB 显卡） |
|---|---|---|---|---|
| 城市低速，一次性路线 | 40 | 50 | 250 | 4000 |
| 高速或城际 | 80–100 | 80–100 | 300 | 4500 |
| 园区或停车场反复绕圈 | 40 | 50 | 400–500 | 4000 |
| 显存紧张（8 GB） | 20–30 | 20–30 | 200 | 2000–3000 |

## 监控

全局建图每 100 个 submap 打印一次：

```
GPU offload: 120/1500 submaps on GPU (850 MB), device memory usage 5200/12282 MB (incl. CUDA memory pool)
```

- `submaps on GPU (xxx MB)`：卸载机制管理的 submap 数据量，**用这个值判断卸载是否生效**，它应该稳定在 `gpu_offload_max_resident_mb` 以内。
- `device memory usage`：整张卡的显存占用。释放的显存会留在 CUDA 内存池里给后续的分配复用，所以 `nvidia-smi` 和 `memory_monitor` 看到的数字**只会停止增长，不会下降**。这是正常现象。

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `cudaErrorIllegalAddress` 后崩溃 | 没有安装带修复的 gtsam_points | 按"前置条件"重新安装 |
| `GPU submap data (...) exceed gpu_offload_max_resident_mb ... but no more submaps can be offloaded` | 最新 N 个加上预加载区里的 submap 已经超过预算 | 调小 `keep_recent_submaps` 或 `prefetch_margin`，或者调大预算 |
| 大回环时显存突然冲高 | iSAM2 全量重新线性化（见下方限制） | 暂时只能靠显卡本身的余量 |
| `nvidia-smi` 显存不下降 | CUDA 内存池保留了已释放的显存 | 正常现象，看日志里的 `submaps on GPU` |

## 限制

- **全量重新线性化**：一次大回环影响的变量超过 65% 时，iSAM2 会重新线性化全部因子，所有 submap 会一次性加载回 GPU，这一瞬间的显存峰值和不开卸载时一样。iSAM2 遇到 indeterminant 异常后重建整张图时也一样。按 12 GB 显卡估算，地图超过约 1300 个 submap（约 30 分钟）时，这种情况下仍然可能溢出。
- **`find_overlapping_submaps`**（viewer 里的手动按钮）：会加载 100 m 内所有的 submap 对，中间不卸载。
- **offline_viewer**：`GlobalMapping::load()` 会把所有 submap 上传到 GPU，不经过卸载机制。长地图如果只是为了导出点云，可以把 `<dump>/config/config_global_mapping_gpu.json` 里的 `registration_error_factor_type` 改成 `"VGICP"`，弹窗 "Do optimization?" 选 **No**。`Export Points` 只用 CPU 数据，导出结果完全一样。
- **不兼容同步调用 GPU 因子的模块**：比如 `global_mapping_pose_graph` 和 glim_ext 的 scan context 回环检测。已卸载的数据只有在批量线性化时才会自动重新加载，启用这些模块时不要同时开启 GPU 卸载。
