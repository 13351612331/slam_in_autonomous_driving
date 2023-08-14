//
// Created by kangyu on 2023/8/11.
//
#include "common/math_utils.h"
#include "pangolin_window.h"
#include <gflags/gflags.h>

// 本节程序演示一个正在作圆周运动的车辆
// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进速度 m/s");
DEFINE_bool(use_quaternion, true, "是否使用四元数计算");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 可视化
  sad::ui::PangolinWindow ui;
  if (!ui.Init()) {
    return -1;
  }

  // 弧度制角速度
  double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;
  SE3 pose;                                  // TWB表示的位姿
  Vec3d omega(0, 0, angular_velocity_rad);   // 角速度矢量
  Vec3d v_body(FLAGS_linear_velocity, 0, 0); // 本体系速度
  const double dt = 0.05;                    // 每次更新的时间

  while (!ui.ShouldQuit()) {
    // 更新自身的位置
    Vec3d v_world = pose.so3() * v_body;
    pose.translation() += v_world * dt;

    // 更新自身旋转
    if (FLAGS_use_quaternion) {
      Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt,
                                               0.5 * omega[1] * dt,
                                               0.5 * omega[2] * dt);
      q.normalize();
      pose.so3() = SO3(q);
    } else {
      pose.so3() = pose.so3() * SO3::exp(omega * dt);
    }

    std::cout << "pose: " << pose.translation().transpose() << std::endl;
    ui.UpdateNavState(sad::NavStated(0, pose, v_world));

    usleep(dt * 1e6);
  }
  ui.Quit();
  return 0;
}