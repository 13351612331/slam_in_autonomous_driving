//
// Created by kangyu on 2023/8/11.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_EIGEN_TYPES_H
#define SLAM_IN_AUTONOMOUS_DRIVING_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

using Vec2i = Eigen::Vector2i;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;

using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat15d = Eigen::Matrix<double, 15, 15>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;
namespace sad {
// 矢量比较
template <int N> struct less_vec {
  inline bool operator()(const Eigen::Matrix<int, N, 1> &v1,
                         const Eigen::Matrix<int, N, 1> &v2) const;
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v1,
                                    const Eigen::Matrix<int, 2, 1> &v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_EIGEN_TYPES_H
