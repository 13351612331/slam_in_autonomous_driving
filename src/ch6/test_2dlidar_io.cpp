//
// Created by kangyu on 2024/4/14.
//
#include "common/io_utils.h"
#include "common/lidar_utils.h"
#include "lidar_2d_utils.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv4/opencv2/highgui.hpp>

DEFINE_string(bag_path, "./dataset/sad/2dmapping/test_2d_lidar.bag",
              "数据包路径");

// 测试从rosbag中读取2d scan并plot的结果

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold == google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [](Scan2d::Ptr scan) -> bool {
                         cv::Mat image;
                         sad::Visualize2DScan(scan, SE2(), image,
                                              Vec3b(255, 0, 0));
                         cv::imshow("scan", image);
                         cv::waitKey(20);
                         return true;
                       })
      .Go();

  return 0;
}