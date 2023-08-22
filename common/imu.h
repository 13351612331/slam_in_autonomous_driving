//
// Created by kangyu on 2023/8/21.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_IMU_H
#define SLAM_IN_AUTONOMOUS_DRIVING_IMU_H

#include <memory>

namespace sad {
// IMU读数
struct IMU {
  IMU() = default;
  IMU(double t, const Vec3d &gyro, const Vec3d &acce)
      : timestamp_(t), gyro_(gyro), acce_(acce) {}

  double timestamp_ = 0.0;
  Vec3d gyro_ = Vec3d::Zero();
  Vec3d acce_ = Vec3d::Zero();
};
} // namespace sad

using IMUPtr = std::shared_ptr<sad::IMU>;

#endif // SLAM_IN_AUTONOMOUS_DRIVING_IMU_H
