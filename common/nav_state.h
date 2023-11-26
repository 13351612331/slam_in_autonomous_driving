//
// Created by kangyu on 2023/8/14.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_NAV_STATE_H
#define SLAM_IN_AUTONOMOUS_DRIVING_NAV_STATE_H

#include "eigen_types.h"

namespace sad {
/**
 * 导航用的状态变量
 * @tparam T 变量类型
 *
 * 这是个封装好的类，部分程序使用本结构体，也有一部分程序使用散装的pvq，都可以的
 */
template <typename T> struct NavState {
  using Vec3 = Eigen::Matrix<T, 3, 1>;
  using SO3 = Sophus::SO3<T>;

  NavState() = default;

  //  from time , R , p , v , bg , ba
  explicit NavState(double time, const SO3 &R = SO3(),
                    const Vec3 &t = Vec3::Zero(), const Vec3 &v = Vec3::Zero(),
                    const Vec3 &bg = Vec3::Zero(),
                    const Vec3 &ba = Vec3::Zero())
      : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {}

  // from pose and vel
  NavState(double time, const SE3 &pose, const Vec3 &vel = Vec3::Zero())
      : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

  // 转换到Sophus
  Sophus::SE3<T> GetSE3() const { return SE3(R_, p_); }

  double timestamp_ = 0;   // 时间
  SO3 R_;                  // 旋转
  Vec3 p_ = Vec3::Zero();  // 平移
  Vec3 v_ = Vec3::Zero();  // 速度
  Vec3 bg_ = Vec3::Zero(); // gyro零偏
  Vec3 ba_ = Vec3::Zero(); // acce 零便
};

using NavStated = NavState<double>;
using NavStatef = NavState<float>;
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_NAV_STATE_H
