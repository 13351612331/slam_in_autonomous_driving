//
// Created by kangyu on 2023/8/22.
//
#include "io_utils.h"
#include <iostream>
#include <sstream>

namespace sad {
void TxtIO::Go() {
  if (!fin) {
    std::cerr << "未能找到文件" << std::endl;
    return;
  }

  while (!fin.eof()) {
    std::string line;
    std::getline(fin, line);
    if (line.empty()) {
      continue;
    }

    if (line[0] == '#') {
      // 以#开头的是注释
      continue;
    }

    // load data from line
    std::stringstream ss;
    ss << line;
    std::string data_type;
    ss >> data_type;

    if (data_type == "IMU" && imu_proc_) {
      double time, gx, gy, gz, ax, ay, az;
      ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
      imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
    }
  }
  std::cout << "done." << std::endl;
}
} // namespace sad