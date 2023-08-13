//
// Created by kangyu on 2023/8/11.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_UI_TRAJECTORY_H
#define SLAM_IN_AUTONOMOUS_DRIVING_UI_TRAJECTORY_H

#include "common/eigen_types.h"
#include <pangolin/gl/glvbo.h>

namespace sad::ui {
// UI中的轨迹绘制
class UiTrajectory {
public:
  explicit UiTrajectory(const Vec3f &color) : color_(color) {
    pos_.reserve(max_size);
  }

  // 增加一个轨迹点
  void AddPt(const SE3 &pose);

  // 渲染此轨迹
  void Render();

private:
  int max_size = 1e6;           // 记录的最大点数
  std::vector<Vec3f> pos_;      // 轨迹记录数据
  Vec3f color_ = Vec3f::Zero(); // 轨迹显示颜色
  pangolin::GlBuffer vbo_;      // 显存顶点信息
};
} // namespace sad::ui

#endif // SLAM_IN_AUTONOMOUS_DRIVING_UI_TRAJECTORY_H
