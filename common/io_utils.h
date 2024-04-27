//
// Created by kangyu on 2023/8/21.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H

#include "common/dataset_type.h"
#include "common/eigen_types.h"
#include "common/global_flags.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/odom.h"
#include <fstream>
#include <functional>
#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
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

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
public:
  explicit RosbagIO(std::string bag_file,
                    DatasetType dataset_type = DatasetType::NCLT)
      : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
    assert(dataset_type_ != DatasetType::UNKNOWN);
  }

  using MessageProcessFunction =
      std::function<bool(const rosbag::MessageInstance &m)>;

  // 一些方便直接使用的topics，messages
  using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanPtr)>;

  // 遍历文件内容，调用回调函数
  void Go();

  // 通用处理函数
  RosbagIO &AddHandle(const std::string &topic_name,
                      MessageProcessFunction func) {
    process_func_.emplace(topic_name, func);
    return *this;
  }
  // 2D激光处理
  RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
    return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
      auto msg = m.instantiate<sensor_msgs::LaserScan>();
      if (msg == nullptr) {
        return false;
      }
      return f(msg);
    });
  }

private:
  std::map<std::string, MessageProcessFunction> process_func_;
  std::string bag_file_;
  DatasetType dataset_type_;
};
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_IO_UTILS_H
