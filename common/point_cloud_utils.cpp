//
// Created by kangyu on 2023/12/26.
//
#include "point_cloud_utils.h"
#include <pcl/filters/voxel_grid.h>

// 点云的一些工具函数

namespace sad {
// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size) {
  pcl::VoxelGrid<sad::PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);

  CloudPtr output(new PointCloudType);
  voxel.filter(*output);
  cloud->swap(*output);
}
} // namespace sad