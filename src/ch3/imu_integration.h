#include <utility>

//
// Created by kangyu on 2023/8/21.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_IMU_INTEGRATION_H
#define SLAM_IN_AUTONOMOUS_DRIVING_IMU_INTEGRATION_H

#include "common/nav_state.h"

namespace sad {
/**
 * 本程序演示单纯靠IMU的积分
 */
class IMUIntegration {
public:
  IMUIntegration(Vec3d gravity, Vec3d init_bg, Vec3d init_ba)
      : gravity_(std::move(gravity)), bg_(std::move(init_bg)),
        ba_(std::move(init_ba)) {}

  // 增加IMU读数
  void AddIMU(const IMU &imu) {
    double dt = imu.timestamp_ - timestamp_;
    if (dt > 0 && dt < 0.1) {
      // 假设IMU时间间隔在0至0.1以内
      p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt +
           0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
      v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
      R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
    }

    // 更新时间
    timestamp_ = imu.timestamp_;
  }

  // 组成NavState
  NavStated GetNavState() const {
    return NavStated(timestamp_, R_, p_, v_, bg_, ba_);
  }

  [[nodiscard]] SO3 GetR() const { return R_; }
  [[nodiscard]] Vec3d GetV() const { return v_; }
  [[nodiscard]] Vec3d GetP() const { return p_; }

private:
  // 累计量
  SO3 R_;
  Vec3d v_ = Vec3d::Zero();
  Vec3d p_ = Vec3d::Zero();
  double timestamp_ = 0.0;
  // 零偏，由外部设定
  Vec3d bg_ = Vec3d ::Zero();
  Vec3d ba_ = Vec3d ::Zero();

  Vec3d gravity_ = Vec3d(0, 0, -9.8); // 重力
};
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_IMU_INTEGRATION_H
