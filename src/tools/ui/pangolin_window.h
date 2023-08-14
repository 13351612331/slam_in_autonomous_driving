//
// Created by kangyu on 2023/8/11.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_H
#define SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_H
#include <Eigen/Core>
#include <iostream>
#include <memory>

#include "common/nav_state.h"
#include "pangolin_window_impl.h"

namespace sad::ui {
class PangolinWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PangolinWindow();
  ~PangolinWindow();

  /**
   * @brief 初始化窗口，后台启动render线程
   * @note 与OpenGL/Pangolin无关的初始化，尽量放到此函数体中；
   *        OpenGL/Pangolin相关的内容，尽量放到PangolinWindowImpl::Init中
   */
  bool Init();

  // 更新kalman滤波器状态
  void UpdateNavState(const NavStated &state);

  // 等待显示线程退出，并释放资源
  void Quit();

  // 用户是否已经退出UI
  bool ShouldQuit();

private:
  std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};
} // namespace sad::ui

#endif // SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_H
