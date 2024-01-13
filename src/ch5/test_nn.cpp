//
// Created by kangyu on 2023/12/26.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "gridnn.h"

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

/**
 * 评测最近邻的正确性
 * @param truth 真值
 * @param esti 估计
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>> &truth,
                     const std::vector<std::pair<size_t, size_t>> &esti) {
  int fp = 0; // false-positive, esti存在但truth中不存在
  int fn = 0; // false-negative, truth存在但esti不存在

  LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

  // 检查某个匹配在另一个容器中存不存在
  auto exist = [](const std::pair<size_t, size_t> &data,
                  const std::vector<std::pair<size_t, size_t>> &vec) -> bool {
    return std::find(vec.begin(), vec.end(), data) != vec.end();
  };

  int effective_esti = 0;
  for (const auto &d : esti) {
    if (d.first != sad::math::kINVALID_ID &&
        d.second != sad::math::kINVALID_ID) {
      effective_esti++;

      if (!exist(d, truth)) {
        fp++;
      }
    }
  }

  for (const auto &d : truth) {
    if (!exist(d, esti)) {
      fn++;
    }
  }

  float precision = 1.0 - float(fp) / effective_esti;
  float recall = 1.0 - float(fn) / truth.size();
  LOG(INFO) << "precision: " << precision << ", recall: " << recall
            << ", fp: " << fp << ", fn: " << fn;
}

TEST(CH5_TEST, GRID_NN) {
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

  std::vector<std::pair<size_t, size_t>> truth_matches;
  sad::bfnn_cloud(first, second, truth_matches);

  // 对比不同种类的grid
  sad::GridNN<2> grid0(0.1, sad::GridNN<2>::NearbyType::CENTER),
      grid4(0.1, sad::GridNN<2>::NearbyType::NEARBY4),
      grid8(0.1, sad::GridNN<2>::NearbyType::NEARBY8);
  sad::GridNN<3> grid3(0.1, sad::GridNN<3>::NearbyType::NEARBY6);

  grid0.SetPointCloud(first);
  grid4.SetPointCloud(first);
  grid8.SetPointCloud(first);
  grid3.SetPointCloud(first);

  // 评价各种版本的Grid NN
  LOG(INFO) << "====================";
  std::vector<std::pair<size_t, size_t>> matches;
  sad::evaluate_and_call(
      [&first, &second, &grid0, &matches]() {
        grid0.GetClosestPointForCloud(first, second, matches);
      },
      "Grid0 单线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid0, &matches]() {
        grid0.GetClosestPointForCloudMT(first, second, matches);
      },
      "Grid0 多线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid4, &matches]() {
        grid4.GetClosestPointForCloud(first, second, matches);
      },
      "Grid4 单线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid4, &matches]() {
        grid4.GetClosestPointForCloudMT(first, second, matches);
      },
      "Grid4 多线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid8, &matches]() {
        grid8.GetClosestPointForCloud(first, second, matches);
      },
      "Grid8 单线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid8, &matches]() {
        grid8.GetClosestPointForCloudMT(first, second, matches);
      },
      "Grid8 多线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid3, &matches]() {
        grid3.GetClosestPointForCloud(first, second, matches);
      },
      "Grid3D 单线程", 10);
  EvaluateMatches(truth_matches, matches);

  LOG(INFO) << "====================";
  sad::evaluate_and_call(
      [&first, &second, &grid3, &matches]() {
        grid3.GetClosestPointForCloudMT(first, second, matches);
      },
      "Grid3D 多线程", 10);
  EvaluateMatches(truth_matches, matches);

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