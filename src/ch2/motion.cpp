//
// Created by kangyu on 2023/8/11.
//
#include "pangolin_window.h"
#include <gflags/gflags.h>

// 本节程序演示一个正在作圆周运动的车辆
// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 可视化
  sad::ui::PangolinWindow ui;
  if (!ui.Init()) {
    return -1;
  }
}