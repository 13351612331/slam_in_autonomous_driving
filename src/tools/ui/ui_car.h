//
// Created by kangyu on 2023/8/13.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_UI_CAR_H
#define SLAM_IN_AUTONOMOUS_DRIVING_UI_CAR_H

#include "common/eigen_types.h"
#include <pangolin/gl/glvbo.h>

namespace sad::ui {
// 在UI里显示的小车
class UiCar {
public:
  UiCar(const Vec3f &color) : color_(color) {}

  // 设置小车Pose，重设显存中的点
  void SetPose(const SE3 &pose);

  // 渲染小车
  void Render();

private:
  Vec3f color_;
  pangolin::GlBuffer vbo_; // buffer data

  static std::vector<Vec3f> cat_vertices_; // 小车顶点
};
} // namespace sad::ui

#endif // SLAM_IN_AUTONOMOUS_DRIVING_UI_CAR_H
