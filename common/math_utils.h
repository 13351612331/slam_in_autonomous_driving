//
// Created by kangyu on 2023/8/13.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H

#include <boost/math/tools/precision.hpp>
#include <numeric>

// 常用的数学函数
namespace sad::math {
// 常量定义
constexpr double kDEG2RAD = M_PI / 180.0; // deg->rad

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
} // namespace sad::math

#endif // SLAM_IN_AUTONOMOUS_DRIVING_MATH_UTILS_H
