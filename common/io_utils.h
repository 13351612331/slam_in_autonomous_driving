//
// Created by kangyu on 2023/8/21.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H

#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/odom.h"
#include <fstream>
#include <functional>
#include <string>

namespace sad {
/**
 * 读取本书提供的数据文本文件，并调用回调函数
 * 数据文本文件主要提供IMU/Odom/GNSS读数
 */
class TxtIO {
public:
  TxtIO(const std::string &file_path) : fin(file_path) {}

  // 定义回调函数
  using IMUProcessFuncType = std::function<void(const IMU &)>;
  using OdomProcessFuncType = std::function<void(const Odom &)>;
  using GNSSProcessFuncType = std::function<void(const GNSS &)>;

  TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
    imu_proc_ = std::move(imu_proc);
    return *this;
  }

  TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
    odom_proc_ = std::move(odom_proc);
    return *this;
  }

  TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
    gnss_proc_ = gnss_proc;
    return *this;
  }

  // 遍历文件内容，调用回调函数
  void Go();

private:
  std::ifstream fin;
  IMUProcessFuncType imu_proc_;
  OdomProcessFuncType odom_proc_;
  GNSSProcessFuncType gnss_proc_;
};
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H
