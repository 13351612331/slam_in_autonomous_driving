//
// Created by kangyu on 2024/4/14.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_DATASET_TYPE_H
#define SLAM_IN_AUTONOMOUS_DRIVING_DATASET_TYPE_H

namespace sad {
// 枚举用到的一些数据集
enum class DatasetType {
  UNKNOWN = -1,
  NCLT = 0,  // NCLT: http://robots.engin.umich.edu/nclt/
  KITTI = 1, // Kitti:
  ULHK = 3,  // https://github.com/weisongwen/UrbanLoco
  UTBM = 4,  // https://epan-utbm.github.io/utbm_robocar_dataset/
  AVIA = 5,  // https://epan-utbm.github.io/utbm_robocar_dataset/
  WXB_3D,    // 3d wxb
};
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_DATASET_TYPE_H
