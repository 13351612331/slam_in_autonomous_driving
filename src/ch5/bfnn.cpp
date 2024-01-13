//
// Created by kangyu on 2023/12/20.
//
#include "bfnn.h"
#include <execution>

namespace sad {
int bfnn_point(CloudPtr cloud, const Vec3f &point) {
  return std::min_element(
             cloud->points.begin(), cloud->points.end(),
             [&point](const PointType &pt1, const PointType &pt2) -> bool {
               return (pt1.getVector3fMap() - point).squaredNorm() <
                      (pt2.getVector3fMap() - point).squaredNorm();
             }) -
         cloud->points.begin();
}

void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2,
                   std::vector<std::pair<size_t, size_t>> &matches) {
  // 生成索引
  std::vector<size_t> index(cloud2->size());
  std::for_each(index.begin(), index.end(),
                [idx = 0](size_t &i) mutable { i = idx++; });

  // 并行化for_each
  matches.resize(index.size());
  std::for_each(
      std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
      });
}

void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2,
                std::vector<std::pair<size_t, size_t>> &matches) {
  // 单线程版本
  std::vector<size_t> index(cloud2->size());
  std::for_each(index.begin(), index.end(),
                [idx = 0](size_t &i) mutable { i = idx++; });

  matches.resize(index.size());
  std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
    matches[idx].second = idx;
    // std::cout << "bfnn cloud begin 4 " << idx << std::endl;
    matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
  });
}
} // namespace sad