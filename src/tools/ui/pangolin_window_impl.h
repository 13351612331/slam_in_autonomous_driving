//
// Created by kangyu on 2023/8/11.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_IMPL_H
#define SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_IMPL_H

#include <Eigen/Core>
#include <atomic>
#include <pangolin/display/default_font.h>
#include <pangolin/pangolin.h>
#include <thread>

#include "common/point_types.h"
#include "ui_car.h"
#include "ui_cloud.h"
#include "ui_trajectory.h"

namespace sad::ui {
class PangolinWindowImpl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PangolinWindowImpl() = default;
  ~PangolinWindowImpl() = default;

  PangolinWindowImpl(const PangolinWindowImpl &) = delete;
  PangolinWindowImpl &operator=(const PangolinWindowImpl &) = delete;
  PangolinWindowImpl(PangolinWindowImpl &&) = delete;
  PangolinWindowImpl &operator=(PangolinWindowImpl &&) = delete;

  // 初始化，创建各种点云、小车实体
  bool Init();

  // 注销
  bool DeInit();

  // 渲染所有信息
  void Render();

private:
  // 创建OpenGL Buffers
  void AllocateBuffer();
  // 释放OpenGL Buffers
  void ReleaseBuffer();

  void CreateDisplayLayout();

  void DrawAll(); // 作图：画定位窗口

  // 渲染点云，调用各种Update函数
  void RenderClouds();
  bool UpdateGps();
  bool UpdateGlobalMap();
  bool UpdateState();
  bool UpdateCurrentScan();

  void RenderLabels();

public:
  // 后台渲染线程
  std::thread render_thread_;

  // 一些辅助锁和原子变量
  std::mutex mtx_map_cloud_;
  std::mutex mtx_current_scan_;
  std::mutex mtx_nav_state_;
  std::mutex mtx_gps_pose_;

  std::atomic<bool> exit_flag_;
  std::atomic<bool> cloud_global_need_update_;
  std::atomic<bool> kf_result_need_update_;
  std::atomic<bool> current_scan_need_update_;
  std::atomic<bool> lidarloc_need_update_;
  std::atomic<bool> pgoloc_need_update_;
  std::atomic<bool> gps_need_update_;

  CloudPtr current_scan_ = nullptr; // 当前scan
  SE3 current_pose_;                // 当前scan对应的pose

  // 地图点云
  std::map<Vec2i, CloudPtr, less_vec<2>> cloud_global_map_;

  // gps
  SE3 gps_pose_;

  // 滤波器状态
  SE3 pose_;
  Vec3d vel_;
  Vec3d bias_acc_;
  Vec3d bias_gyr_;
  Vec3d grav_;

  int max_size_of_current_scan_ = 2000; // 当前扫描数据保留多少个

private:
  // 窗口layout相关
  int win_width_ = 1920;
  int win_height_ = 1080;
  static constexpr float cam_focus_ = 5000;
  static constexpr float cam_z_near_ = 1.0;
  static constexpr float cam_z_far_ = 1e10;
  static constexpr int menu_width_ = 200;
  const std::string win_name_ = "SAD.UI";
  const std::string dis_main_name_ = "main";
  const std::string dis_3d_name_ = "Cam 3D";
  const std::string dis_3d_main_name_ = "Cam 3D Main";
  const std::string dis_plot_name_ = "Plot";

  bool following_loc_ = true; // 相机是否追踪定位结果

  // text
  pangolin::GlText gltext_label_global_;

  // camera
  pangolin::OpenGlRenderState s_cam_main_;

  // cloud rendering
  ui::UiCar car_{Vec3f(0.2, 0.2, 0.0)};
  std::map<Vec2i, std::shared_ptr<ui::UiCloud>, less_vec<2>>
      cloud_map_ui_;                               // 用来渲染点云地图
  std::shared_ptr<ui::UiCloud> current_scan_ui_;   // current scan
  std::deque<std::shared_ptr<ui::UiCloud>> scans_; // current scan 保留队列

  // trajectory
  std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;
  std::shared_ptr<ui::UiTrajectory> traj_gps_ui_ = nullptr;

  // 滤波器状态相关 Data logger object
  pangolin::DataLog log_vel_;          // odom frame下的速度
  pangolin::DataLog log_vel_baselink_; // baselink frame下的速度
  pangolin::DataLog log_bias_acc_;
  pangolin::DataLog log_bias_gyr_;

  std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
  std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
  std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
  std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
};
} // namespace sad::ui
#endif // SLAM_IN_AUTONOMOUS_DRIVING_PANGOLIN_WINDOW_IMPL_H
