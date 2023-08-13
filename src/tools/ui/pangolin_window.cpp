//
// Created by kangyu on 2023/8/11.
//
#include "pangolin_window.h"

namespace sad::ui {
PangolinWindow::PangolinWindow() {
  impl_ = std::make_shared<PangolinWindowImpl>();
}

PangolinWindow::~PangolinWindow() {
  std::cout << "Pangolin window is deallocated.";
  Quit();
}
void PangolinWindow::Quit() {
  if (impl_->render_thread_.joinable()) {
    impl_->exit_flag_.store(true);
    impl_->render_thread_.join();
  }
  impl_->DeInit();
}
bool PangolinWindow::Init() {
  impl_->cloud_global_need_update_.store(false);
  impl_->kf_result_need_update_.store(false);
  impl_->lidarloc_need_update_.store(false);
  impl_->pgoloc_need_update_.store(false);
  impl_->gps_need_update_.store(false);
  impl_->current_scan_need_update_.store(false);

  bool inited = impl_->Init();
  if (inited) {
    impl_->render_thread_ = std::thread([this]() { impl_->Render(); });
  }
  return inited;
}
} // namespace sad::ui