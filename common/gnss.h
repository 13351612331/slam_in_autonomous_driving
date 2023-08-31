//
// Created by kangyu on 2023/8/24.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_GNSS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_GNSS_H

#include "common/eigen_types.h"

namespace sad {

// GNSS状态位信息
// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
  GNSS_FLOAT_SOLUTION = 5,        // 浮点解（cm到dm之间）
  GNSS_FIXED_SOLUTION = 4,        // 固定解（cm级）
  GNSS_PSEUDO_SOLUTION = 2,       // 伪距差分解（分米级）
  GNSS_SINGLE_POINT_SOLUTION = 1, // 单点解（10cm级）
  GNSS_NOT_EXIST = 0,             // GPS无信号
  GNSS_OTHER = -1,                // 其他
};
// UTM坐标系
struct UTMCoordinate {
  UTMCoordinate() = default;
  explicit UTMCoordinate(int zone, const Vec2d &xy = Vec2d::Zero(),
                         bool north = true) {}

  int zone_ = 0;             // utm区域
  Vec2d xy_ = Vec2d::Zero(); // UTM xy
  double z_ = 0;             // z 高度（直接来自于GPS）
  bool north_ = true;        // 是否在北半球
};

// 一个GNSS读数结构
struct GNSS {
  GNSS() = default;
  GNSS(double unix_time, int status, const Vec3d &lat_lon_alt, double heading,
       bool heading_valid)
      : unix_time_(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading),
        heading_valid_(heading_valid) {
    status_ = GpsStatusType(status);
  }

  double unix_time_ = 0;                                 // unix系统时间
  GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST; // GNSS状态位
  Vec3d lat_lon_alt_ = Vec3d::Zero(); // 经度、纬度、高度、前二者单位为度
  double heading_ = 0.0;       // 双天线读到的方位角，单位为度
  bool heading_valid_ = false; // 方位角是否有效

  // UTM坐标（区域之类的也在内）
  UTMCoordinate utm_;
  // UTM坐标是否已经计算（若经纬度给出错误数值，此处也为false）
  bool utm_valid_ = false;

  SE3 utm_pose_; // 用于后处理的6DoF Pose
};
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_GNSS_H
