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
using IndexVec = std::vector<int>;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType &pt) { return pt.getVector3fMap(); }

// 模板类型转换函数
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt);

template <> inline Eigen::Matrix<float, 2, 1> ToEigen(const PointType &pt) {
  return Vec2f(pt.x, pt.y);
}

template <> inline Eigen::Matrix<float, 3, 1> ToEigen(const PointType &pt) {
  return Vec3f(pt.x, pt.y, pt.z);
}
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_POINT_TYPES_H
