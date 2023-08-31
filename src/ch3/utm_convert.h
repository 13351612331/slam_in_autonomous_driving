//
// Created by kangyu on 2023/8/24.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_UTM_CONVERT_H
#define SLAM_IN_AUTONOMOUS_DRIVING_UTM_CONVERT_H
#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/math_utils.h"
namespace sad {

/**
 * 计算本书的GNSS读数对应的UTM pose和六自由度Pose
 * @param gnss_reading 输入gnss读数
 * @param antenna_pos 安装位置
 * @param antenna_angle 安装偏角
 * @param map_origin 地图原点，指定时，将从UTM位置中减掉坐标原点
 * @return
 */
bool ConvertGps2UTM(GNSS &gnss_reading, const Vec2d &antenna_pos,
                    const double &antenna_angle,
                    const Vec3d &map_origin = Vec3d::Zero());

/**
 * 经纬度转UTM
 * NOTE 经纬度单位为度数
 */
bool LatLon2UTM(const Vec2d &latlon, UTMCoordinate &utm_coor);
} // namespace sad

#endif // SLAM_IN_AUTONOMOUS_DRIVING_UTM_CONVERT_H
