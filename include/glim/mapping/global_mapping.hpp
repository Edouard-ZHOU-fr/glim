#pragma once

#include <any>
#include <cstdint>
#include <memory>
#include <random>
#include <glim/mapping/global_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
struct ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;

/**
 * @brief Global mapping parameters
 */
struct GlobalMappingParams {
public:
  GlobalMappingParams();
  ~GlobalMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  bool enable_between_factors;
  std::string between_registration_type;

  std::string registration_error_factor_type;
  double submap_voxel_resolution;
  double submap_voxel_resolution_max;
  double submap_voxel_resolution_dmin;
  double submap_voxel_resolution_dmax;
  int submap_voxelmap_levels;
  double submap_voxelmap_scaling_factor;

  double randomsampling_rate;
  double max_implicit_loop_distance;
  double min_implicit_loop_overlap;

  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  double init_pose_damping_scale;

  // GPU memory offloading (long-term mapping)
  bool enable_gpu_offload;                  ///< Offload GPU data of distant submaps to CPU memory
  int gpu_offload_keep_recent_submaps;      ///< Number of latest submaps always kept on the GPU
  double gpu_offload_prefetch_margin;       ///< Submaps within (max_implicit_loop_distance + margin) are (re)loaded on the GPU
  double gpu_offload_distance;              ///< Submaps farther than this distance are offloaded
  double gpu_offload_max_resident_mb;       ///< Max GPU memory [MB] for submap data; least recently used submaps are additionally offloaded beyond it (<= 0: disabled)
  bool gpu_offload_freeze_relinearization;  ///< Do not relinearize offloaded submaps (avoid reloading them in iSAM2 updates)
};

/**
 * @brief Global mapping
 */
class GlobalMapping : public GlobalMappingBase {
public:
  GlobalMapping(const GlobalMappingParams& params = GlobalMappingParams());
  virtual ~GlobalMapping();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;

  virtual void find_overlapping_submaps(double min_overlap) override;
  virtual void optimize() override;

  virtual void save(const std::string& path) override;
  virtual gtsam_points::PointCloud::Ptr export_points() override;

  /**
   * @brief Load a mapping result from a dumped directory
   * @param path Input dump path
   */
  bool load(const std::string& path);

private:
  void insert_submap(int current, const SubMap::Ptr& submap);

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_between_factors(int current) const;
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) const;

  void update_submaps();
  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values);

  // GPU memory offloading
  bool gpu_offload_enabled() const;
  bool submap_loaded_on_gpu(int i) const;
  size_t submap_memory_usage_gpu(int i) const;
  std::uint64_t submap_last_accessed_time(int i) const;
  void touch_submap(int i) const;
  bool offload_submap(int i) const;
  void manage_gpu_memory();
  bool evict_submaps_from_gpu(const std::vector<int>& submaps_in_use) const;
  template <typename Func>
  void retry_on_gpu_oom(const std::vector<int>& submaps_in_use, const Func& func) const;

  // Error handling
  void halt();

  void recover_graph() override;
  std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> recover_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, int start_from_frame_id) const;

private:
  using Params = GlobalMappingParams;
  Params params;

  std::mt19937 mt;
  int session_id;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::any stream_buffer_roundrobin;

  std::vector<SubMap::Ptr> submaps;
  std::vector<gtsam_points::PointCloud::ConstPtr> subsampled_submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::uint64_t gpu_offload_access_time;  ///< GPU data access counter at the last manage_gpu_memory() call

  // Error handling (e.g., GPU out of memory during an iSAM2 update, which may leave iSAM2 partially updated)
  bool halted;                   ///< If true, further processing is skipped and save() writes only the committed state
  bool isam2_partially_updated;  ///< If true, an iSAM2 update failed in the middle and its estimate must not be used
  size_t num_committed_factors;  ///< Number of iSAM2 factors after the last successful update

  std::shared_ptr<void> tbb_task_arena;
};
}  // namespace glim
