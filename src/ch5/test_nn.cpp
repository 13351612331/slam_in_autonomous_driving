//
// Created by kangyu on 2023/12/26.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "bfnn.h"
#include "common/point_cloud_utils.h"
#include "common/sys_utils.h"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "ANN的比例因子");

TEST(CH5_TEST, BFNN) {
  sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
  pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
  pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

  if (first->empty() || second->empty()) {
    LOG(ERROR) << "cannot load cloud";
    FAIL();
  }

  // voxel grid 至 0.05
  sad::VoxelGrid(first);
  sad::VoxelGrid(second);

  LOG(INFO) << "points: " << first->size() << ", " << second->size();

  // 评价单线程和多线程版本的暴力匹配
  sad::evaluate_and_call(
      [&first, &second]() {
        std::vector<std::pair<size_t, size_t>> matches;
        sad::bfnn_cloud(first, second, matches);
      },
      "暴力匹配（单线程）", 1);
  sad::evaluate_and_call(
      [&first, &second]() {
        std::vector<std::pair<size_t, size_t>> matches;
        sad::bfnn_cloud_mt(first, second, matches);
      },
      "暴力匹配（多线程）", 1);

  SUCCEED();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}