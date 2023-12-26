//
// Created by kangyu on 2023/12/26.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_POINT_CLOUD_UTILS_H
#define SLAM_IN_AUTONOMOUS_DRIVING_POINT_CLOUD_UTILS_H

#include "point_types.h"

// 点云的一些工具函数

namespace sad {

// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_POINT_CLOUD_UTILS_H
