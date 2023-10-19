//
// Created by kangyu on 2023/10/19.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_ODOM_H
#define SLAM_IN_AUTONOMOUS_DRIVING_ODOM_H

namespace sad {

struct Odom {
  Odom() {}
  Odom(double timestamp, double left_pulse, double right_pulse)
      : timestamp_(timestamp), left_pulse_(left_pulse),
        right_pulse_(right_pulse) {}

  double timestamp_ = 0.0;
  double left_pulse_ = 0.0; // 左右轮的单位时间转过的脉冲数
  double right_pulse_ = 0.0;
};

} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_ODOM_H
