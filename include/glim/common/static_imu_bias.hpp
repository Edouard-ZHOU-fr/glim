#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

/**
 * @brief Static detection parameters for static IMU bias estimation
 */
struct StaticIMUBiasParams {
public:
  StaticIMUBiasParams();

public:
  double acc_std_thresh;   ///< Threshold for the max per-axis std of linear acc [m/s^2]
  double gyro_std_thresh;  ///< Threshold for the max per-axis std of angular vel [rad/s]
  double max_translation;  ///< Threshold for the sensor translation in the window [m]
  double max_rotation;     ///< Threshold for the sensor rotation in the window [rad]
};

/**
 * @brief Static IMU bias estimation result
 */
struct StaticIMUBiasEstimate {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class Status { INSUFFICIENT_IMU, NOT_STATIC, STATIC };

  Status status;
  int num_imu;       ///< Number of IMU data in the window
  int num_acc;       ///< Number of acc data used for the bias estimation (after outlier rejection)
  int num_gyro;      ///< Number of gyro data used for the bias estimation (after outlier rejection)
  double imu_begin;  ///< Stamp of the first IMU data in the window
  double imu_end;    ///< Stamp of the last IMU data in the window
  double acc_std;    ///< Max per-axis std of linear acc [m/s^2]
  double gyro_std;   ///< Max per-axis std of angular vel [rad/s]
  double max_trans;  ///< Max sensor translation in the window [m]
  double max_rot;    ///< Max sensor rotation in the window [rad]

  Eigen::Matrix<double, 6, 1> imu_bias;  ///< IMU bias [acc, gyro] (valid only if status == STATIC)
  Eigen::Vector3d gravity_dir;           ///< Gravity (up) direction in the IMU frame (valid only if status == STATIC)
};

/**
 * @brief Estimate the IMU bias assuming that the sensor is static in [t_begin, t_end]
 *        Gyro bias = mean angular velocity
 *        Acc bias  = mean acc - gravity. As the gravity direction is taken from the mean acc itself,
 *                    only the component along the gravity is observable (horizontal components are set to zero)
 * @param params    Static detection parameters
 * @param poses     Sensor poses in the window (e.g., given by LiDAR odometry) used for the motion check
 * @param imu_data  IMU data [stamp, acc, gyro]
 * @param t_begin   Window begin time
 * @param t_end     Window end time
 * @param gravity   Gravity magnitude used in the IMU integration
 */
StaticIMUBiasEstimate estimate_static_imu_bias(
  const StaticIMUBiasParams& params,
  const std::vector<Eigen::Isometry3d>& poses,
  const std::deque<Eigen::Matrix<double, 7, 1>>& imu_data,
  double t_begin,
  double t_end,
  double gravity);

}  // namespace glim
