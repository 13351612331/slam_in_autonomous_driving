//
// Created by kangyu on 2023/11/5.
//

#ifndef SLAM_IN_AUTONOMOUS_DRIVING_G2O_TYPES_H
#define SLAM_IN_AUTONOMOUS_DRIVING_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/g2o_core_api.h>

#include "eigen_types.h"
#include "nav_state.h"

namespace sad {
/**
 * 旋转在前的SO3+t类型pose，6自由度，存储时伪装为g2o::VertexSE3,供g2o_viewer查看
 */
class VertexPose : public g2o::BaseVertex<6, SE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose(){};

  bool read(std::istream &is) override {
    double data[7];
    for (int i = 0; i < 7; i++) {
      is >> data[i];
    }
    setEstimate(SE3(Quatd(data[6], data[3], data[4], data[5]),
                    Vec3d(data[0], data[1], data[2])));
    return true;
  }

  bool write(std::ostream &os) const override {
    os << "VERTEX_SE3:QUAT ";
    os << id() << " ";
    Quatd q = _estimate.unit_quaternion();
    os << _estimate.translation().transpose() << " ";
    // q.coeffs() 返回四元数对象的系数
    os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " "
       << q.coeffs()[3] << std::endl;
    return true;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double *update_) {
    _estimate.so3() =
        _estimate.so3() *
        SO3::exp(Eigen::Map<const Vec3d>(&update_[0])); // 旋转部分
    _estimate.translation() += Eigen::Map<const Vec3d>(&update_[3]); // 平移部分
    // updateCache
    // 函数通常用于在优化过程中更新顶点或边的缓存数据，以便进行更高效的优化计算。
    // 具体来说，在 g2o
    // 中，每个顶点和边都可以维护一些额外的数据结构，这些数据结构可以在优化过程中使用。updateCache
    // 函数被用来更新这些数据结构，以确保它们与当前优化变量的数值状态保持同步。
    updateCache();
  }
};

/**
 * 速度顶点，单纯的Vec3d
 */
class VertexVelocity : public g2o::BaseVertex<3, Vec3d> {
public:
  // 这个宏的作用是确保类的实例在内存中按照 Eigen 的对齐要求进行分配和释放。
  // 在使用 Eigen 进行开发时，特别是在定义包含 Eigen
  // 对象成员的自定义类时，我们需要确保这些类的实例在内存中按照 Eigen
  // 的对齐要求进行分配。否则，在某些情况下，由于内存对齐问题可能导致程序崩溃或性能下降。
  // 通过在自定义类的定义中包含 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 宏，我们可以简单地确保该类的实例在内存中按照 Eigen
  // 的对齐要求进行分配。这个宏会自动为我们重载 operator new 和 operator
  // delete，并使用 Eigen 提供的对齐内存分配器。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexVelocity(){};

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  // 设置被优化顶点的初始值
  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual void oplusImpl(const double *update_) {
    _estimate += Eigen::Map<const Vec3d>(update_);
  }
};

/**
 * 陀螺仪零偏顶点，亦为Vec3d,从速度顶点继承
 */
class VertexGyroBias : public VertexVelocity {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexGyroBias() = default;
};

/**
 * 加速度计零偏顶点，Vec3d,亦从速度顶点继承
 */
class VertexAccBias : public VertexVelocity {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexAccBias(){};
};

/**
 * 陀螺仪随机游走
 */
class EdgeGyroRW
    : public g2o::BaseBinaryEdge<3, Vec3d, VertexGyroBias, VertexGyroBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGyroRW() = default;

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const auto *VG1 = dynamic_cast<const VertexGyroBias *>(_vertices[0]);
    const auto *VG2 = dynamic_cast<const VertexGyroBias *>(_vertices[1]);
    _error = VG2->estimate() - VG1->estimate();
  }

  virtual void linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
};

/**
 * 加速度计随机游走
 */
class EdgeAccRW
    : public g2o::BaseBinaryEdge<3, Vec3d, VertexAccBias, VertexAccBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeAccRW() = default;

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError() {
    const auto *VA1 = dynamic_cast<const VertexAccBias *>(_vertices[0]);
    const auto *VA2 = dynamic_cast<const VertexAccBias *>(_vertices[1]);
    _error = VA2->estimate() - VA1->estimate();
  }

  virtual void linearizeOplus() {
    _jacobianOplusXi = -Mat3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
};

/**
 * 6自由度的GNSS
 * 误差的角度在前，平移在后
 */

class EdgeGNSS : public g2o::BaseUnaryEdge<6, SE3, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGNSS() = default;
  EdgeGNSS(VertexPose *v, const SE3 &obs) {
    setVertex(0, v);
    setMeasurement(obs);
  }

  void computeError() override {
    VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    _error.head<3>() =
        (_measurement.so3().inverse() * v->estimate().so3()).log();
    _error.tail<3>() = v->estimate().translation() - _measurement.translation();
  }

  void linearizeOplus() override {
    VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    // jacobian 6x6
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) =
        (_measurement.so3().inverse() * v->estimate().so3()).jr_inv(); // dR/dR
    _jacobianOplusXi.block<3, 3>(3, 3) = Mat3d::Identity();            // dp/dp
  }

  Mat6d GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

/**
 * 对上一帧IMU pvq bias的先验
 * info 由外部指定，通过时间窗口边缘化给出
 *
 * 顶点顺序：pose,v,bg,ba
 * 残差顺序：R,p,v,bg,ba,15维
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Vec15d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePriorPoseNavState(const NavStated &state, const Mat15d &info);

  virtual bool read(std::istream &is) { return false; }
  virtual bool write(std::ostream &os) const { return false; }

  void computeError();
  virtual void linearizeOplus();

  NavStated state_;
};

/**
 * 3维 轮速计观测边
 * 轮速观测世界速度在自车坐标系下矢量，3维情况下假设自车不会有y和z方向速度
 */
class EdgeEncoder3D : public g2o::BaseUnaryEdge<3, Vec3d, VertexVelocity> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeEncoder3D() = default;

  /**
   * 构造函数需要知道世界系下速度
   * @param v0
   * @param speed
   */
  EdgeEncoder3D(VertexVelocity *v0, const Vec3d &speed) {
    setVertex(0, v0);
    setMeasurement(speed);
  }

  void computeError() override {
    VertexVelocity *v0 = (VertexVelocity *)_vertices[0];
    _error = v0->estimate() - _measurement;
  }

  void linearizeOplus() override { _jacobianOplusXi.setIdentity(); }

  virtual bool read(std::istream &is) { return true; }
  virtual bool write(std::ostream &os) const { return true; }
};

} // namespace sad
// #if 0
#endif // SLAM_IN_AUTONOMOUS_DRIVING_G2O_TYPES_H
