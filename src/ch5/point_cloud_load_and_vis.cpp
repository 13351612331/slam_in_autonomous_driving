//
// Created by kangyu on 2023/12/17.
//
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");

// 本程序可用于显示单个点云，演示PCL的基本用法
// 实际上就是调用了pcl的可视化库，类似于pcl_viewer
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (fLS::FLAGS_pcd_path.empty()) {
    LOG(ERROR) << "pcd file is empty";
    return -1;
  }

  // 读取点云
  PointCloudType::Ptr cloud(new PointCloudType);
  pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

  if (cloud->empty()) {
    LOG(ERROR) << "cannot load is empty";
    return -1;
  }

  LOG(INFO) << "cloud points: " << cloud->size();

  // visualize
  pcl::visualization::PCLVisualizer viewer("cloud viewer");
  pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(
      cloud, "z"); // 使用高度来着色
  viewer.addPointCloud<PointType>(cloud, handle);
  viewer.spin();

  return 0;
}