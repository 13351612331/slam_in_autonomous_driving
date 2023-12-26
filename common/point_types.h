//
// Created by kangyu on 2023/8/11.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_POINT_TYPES_H
#define SLAM_IN_AUTONOMOUS_DRIVING_POINT_TYPES_H

#include "common/eigen_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sad {
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType &pt) { return pt.getVector3fMap(); }
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_POINT_TYPES_H
