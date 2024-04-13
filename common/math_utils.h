//
// Created by kangyu on 2023/8/13.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H

#include "common/eigen_types.h"
#include <boost/math/tools/precision.hpp>
#include <numeric>

// 常用的数学函数
namespace sad::math {
// 常量定义
constexpr double kDEG2RAD = M_PI / 180.0; // deg->rad

// 非法定义
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C 容器类型
 * @tparam D 结果类型
 * @tparam Getter 获取数据函数，接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C &data, D &mean, D &cov_diag,
                           Getter &&getter) {
  size_t len = data.size();
  assert(len > 1);
  mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                         [&getter](const D &sum, const auto &data) -> D {
                           return sum + getter(data);
                         }) /
         len;
  cov_diag =
      std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                      [&mean, &getter](const D &sum, const auto &data) -> D {
                        return sum + (getter(data) - mean).cwiseAbs2().eval();
                      }) /
      (len - 1);
}

template <typename S>
bool FitPlane(std::vector<Eigen::Matrix<S, 3, 1>> &data,
              Eigen::Matrix<S, 4, 1> &plane_coeffs, double eps = 1e-2) {
  if (data.size() < 3) {
    return false;
  }

  Eigen::MatrixXd A(data.size(), 4);
  for (int i = 0; i < data.size(); ++i) {
    A.row(i).head<3>() = data[i].transpose();
    A.row(i)[3] = 1.0;
  }

  Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
  plane_coeffs = svd.matrixV().col(3);

  // check error eps
  for (int i = 0; i < data.size(); ++i) {
    double err = plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3];
    if (err * err > eps) {
      return false;
    }
  }
  return true;
}

template <typename S>
bool FitLine(std::vector<Eigen::Matrix<S, 3, 1>> &data,
             Eigen::Matrix<S, 3, 1> &origin, Eigen::Matrix<S, 3, 1> &dir,
             double eps = 0.2) {
  if (data.size() < 2) {
    return false;
  }

  // 参数1，累加范围的起始
  // 参数2，累加范围的终止
  // 参数3，累加值的初始值
  // 返回值，累加的结果
  origin = std::accumulate(data.begin(), data.end(),
                           Eigen::Matrix<S, 3, 1>::Zero().eval()) /
           data.size();

  Eigen::MatrixXd Y(data.size(), 3);
  for (int i = 0; i < data.size(); ++i) {
    Y.row(i) = (data[i] - origin).transpose();
  }

  Eigen::JacobiSVD svd(Y, Eigen::ComputeFullV);
  dir = svd.matrixV().col(0);

  // check eps
  for (const auto &d : data) {
    if (dir.cross(d - origin).squaredNorm() > eps) {
      return false;
    }
  }

  return true;
}
} // namespace sad::math

#endif // SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H
