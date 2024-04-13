//
// Created by kangyu on 2024/1/22.
//
#include "common/eigen_types.h"
#include "common/math_utils.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

DEFINE_int32(num_tested_points_plane, 10,
             "number of tested points in plane fitting");
DEFINE_int32(num_tested_points_line, 100,
             "number of tested points in line fitting");
DEFINE_double(noise_sigma, 0.01, "noise of generated samples");

void PlaneFittingTest();
void LineFittingTest();

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "testing plane fitting";
  PlaneFittingTest();
  LOG(INFO) << "testing line fitting";
  LineFittingTest();
}

void PlaneFittingTest() {
  Vec4d true_plane_coeffs(0.1, 0.2, 0.3, 0.4);
  true_plane_coeffs.normalize();

  std::vector<Vec3d> points;

  // 随机生成仿真平面点
  cv::RNG rng;
  for (int i = 0; i < FLAGS_num_tested_points_plane; ++i) {
    // 先生成一个随机点，计算第四维，增加噪声，在归一化
    // rng.uniform(a, b) 返回一个[a,b)范围的均匀分布的随机数
    Vec3d p(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0),
            rng.uniform(0.0, 1.0));
    /*
     * 设平面一法向量n，再设平面上一点的位置向量为r0，则对平面上任意一点的位置向量r，
     * 均有：r-r0平行于平面也就是r-r0垂直于n所以有：
     * (r-r0)·n＝0
     * 即：r·n-r0·n＝0
     * 设n＝(A，B，C)，r＝(x，y，z)，r0＝(x0，y0，z0)，
     * 则有：Ax+By+Cz-Ax0-By0-Cz0＝0
     * 所以D＝-(Ax0+By0+Cz0)＝-r0·n
     * 即平面上某点位置向量与平面法向量之内积
     * */
    double n4 = -p.dot(true_plane_coeffs.head<3>()) / true_plane_coeffs[3];
    p = p / (n4 + std::numeric_limits<double>::min()); // 防止除零
    p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma),
               rng.gaussian(FLAGS_noise_sigma));

    points.emplace_back(p);

    // 验证在平面上
    LOG(INFO) << "res of p: "
              << p.dot(true_plane_coeffs.head<3>()) + true_plane_coeffs[3];
  }

  Vec4d estimated_plane_coeffs;
  if (sad::math::FitPlane(points, estimated_plane_coeffs)) {
    LOG(INFO) << "estimated coeffs: " << estimated_plane_coeffs.transpose()
              << " , true: " << true_plane_coeffs.transpose();
  } else {
    LOG(INFO) << "plane fitting failed";
  }
}
void LineFittingTest() {
  // 直线拟合参数真值
  Vec3d true_line_origin(0.1, 0.2, 0.3);
  Vec3d true_line_dir(0.4, 0.5, 0.6);
  true_line_dir.normalize();

  // 随机生成直线点，利用参数方程
  std::vector<Vec3d> points;
  cv::RNG rng;
  for (int i = 0; i < fLI::FLAGS_num_tested_points_line; ++i) {
    double t = rng.uniform(-1.0, 1.0);
    Vec3d p = true_line_origin + true_line_dir * t;
    p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma),
               rng.gaussian(FLAGS_noise_sigma));

    points.emplace_back(p);
  }

  Vec3d esti_origin, esti_dir;
  if (sad::math::FitLine(points, esti_origin, esti_dir)) {
    LOG(INFO) << "estimated origin: " << esti_origin.transpose()
              << " , true: " << true_line_origin.transpose();
    LOG(INFO) << "estimated dir: " << esti_dir.transpose()
              << " , true: " << true_line_dir.transpose();
  } else {
    LOG(INFO) << "Line fitting failed";
  }
}