#include <glim/common/static_imu_bias.hpp>

#include <limits>
#include <gtsam_points/util/running_statistics.hpp>
#include <glim/util/config.hpp>

namespace glim {

StaticIMUBiasParams::StaticIMUBiasParams() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  acc_std_thresh = config.param("odometry_estimation", "static_acc_std_thresh", 0.1);
  gyro_std_thresh = config.param("odometry_estimation", "static_gyro_std_thresh", 0.01);
  max_translation = config.param("odometry_estimation", "static_max_translation", 0.1);
  max_rotation = config.param("odometry_estimation", "static_max_rotation", 0.003);
}

StaticIMUBiasEstimate estimate_static_imu_bias(
  const StaticIMUBiasParams& params,
  const std::vector<Eigen::Isometry3d>& poses,
  const std::deque<Eigen::Matrix<double, 7, 1>>& imu_data,
  double t_begin,
  double t_end,
  double gravity) {
  StaticIMUBiasEstimate estimate;
  estimate.num_acc = 0;
  estimate.num_gyro = 0;
  estimate.imu_bias.setZero();
  estimate.gravity_dir.setZero();

  // Sensor motion in the window
  // (IMU statistics alone cannot reject a constant-rate turn)
  estimate.max_trans = 0.0;
  estimate.max_rot = 0.0;
  for (const auto& pose : poses) {
    const Eigen::Isometry3d delta = poses.front().inverse() * pose;
    estimate.max_trans = std::max(estimate.max_trans, delta.translation().norm());
    estimate.max_rot = std::max(estimate.max_rot, Eigen::AngleAxisd(delta.linear()).angle());
  }

  // Statistics of IMU data [acc, gyro] in the window
  const auto in_window = [&](const Eigen::Matrix<double, 7, 1>& imu) { return imu[0] >= t_begin && imu[0] <= t_end; };

  estimate.imu_begin = std::numeric_limits<double>::max();
  estimate.imu_end = std::numeric_limits<double>::lowest();
  gtsam_points::RunningStatistics<Eigen::Array<double, 6, 1>> imu_stats;
  for (const auto& imu : imu_data) {
    if (in_window(imu)) {
      estimate.imu_begin = std::min(estimate.imu_begin, imu[0]);
      estimate.imu_end = std::max(estimate.imu_end, imu[0]);
      imu_stats.add(imu.tail<6>().array());
    }
  }
  estimate.num_imu = imu_stats.size();

  // IMU data must cover the whole window (IMU may start later than LiDAR)
  if (imu_stats.size() < 10 || estimate.imu_end - estimate.imu_begin < 0.9 * (t_end - t_begin)) {
    estimate.status = StaticIMUBiasEstimate::Status::INSUFFICIENT_IMU;
    estimate.acc_std = estimate.gyro_std = std::numeric_limits<double>::quiet_NaN();
    return estimate;
  }

  const Eigen::Array<double, 6, 1> mean = imu_stats.mean();
  const Eigen::Array<double, 6, 1> stddev = imu_stats.var().max(0.0).sqrt();
  estimate.acc_std = stddev.head<3>().maxCoeff();
  estimate.gyro_std = stddev.tail<3>().maxCoeff();

  const bool is_static = estimate.acc_std < params.acc_std_thresh && estimate.gyro_std < params.gyro_std_thresh &&
                         estimate.max_trans < params.max_translation && estimate.max_rot < params.max_rotation;
  if (!is_static) {
    estimate.status = StaticIMUBiasEstimate::Status::NOT_STATIC;
    return estimate;
  }

  // Reject samples deviating from the mean by more than 2 sigma
  gtsam_points::RunningStatistics<Eigen::Array3d> acc_stats;
  gtsam_points::RunningStatistics<Eigen::Array3d> gyro_stats;
  for (const auto& imu : imu_data) {
    if (!in_window(imu)) {
      continue;
    }

    const Eigen::Array<double, 6, 1> deviation = (imu.tail<6>().array() - mean).abs();
    if ((deviation.head<3>() <= 2.0 * stddev.head<3>()).all()) {
      acc_stats.add(imu.middleRows<3>(1).array());
    }
    if ((deviation.tail<3>() <= 2.0 * stddev.tail<3>()).all()) {
      gyro_stats.add(imu.tail<3>().array());
    }
  }

  const Eigen::Vector3d acc_mean = acc_stats.mean().matrix();
  const Eigen::Vector3d gyro_mean = gyro_stats.mean().matrix();

  estimate.status = StaticIMUBiasEstimate::Status::STATIC;
  estimate.num_acc = acc_stats.size();
  estimate.num_gyro = gyro_stats.size();
  estimate.gravity_dir = acc_mean.normalized();
  estimate.imu_bias << acc_mean - gravity * estimate.gravity_dir, gyro_mean;

  return estimate;
}

}  // namespace glim
