//
// Created by kangyu on 2023/12/20.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_BFNN_H
#define SLAM_IN_AUTONOMOUS_DRIVING_BFNN_H

#include "common/eigen_types.h"
#include "common/point_types.h"

namespace sad {
/**
 * Brute-force Nearest Neighbour
 * @param cloud 点云
 * @param point 待查找点
 * @return 找到的最近点索引
 */
int bfnn_point(CloudPtr cloud, const Vec3f &point);

/**
 * 对点云进行BF最近邻
 * @param cloud1 目标点云
 * @param cloud2 被查找点云
 * @param matches 两个点云内的匹配关系
 */
void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2,
                std::vector<std::pair<size_t, size_t>> &matches);

/**
 * 对点云进行BF最近邻 多线程版本
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2,
                   std::vector<std::pair<size_t, size_t>> &matches);
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_BFNN_H
