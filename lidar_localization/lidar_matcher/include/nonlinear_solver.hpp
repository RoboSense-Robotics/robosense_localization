#pragma once

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/loss_function.h>

#include "se3.hpp"
#include "data_type.hpp"

template <int N>
class Solver {
 public:
  Solver() = default;
  virtual ~Solver(){};
  virtual void addData(const Target& measured_target,
                       const Target& ref_target) = 0;
  virtual bool solve(
      ceres::Solver::Options opts = ceres::Solver::Options()) = 0;
  virtual double getResidual() { return 0; }
  virtual int getNumResiduals() { return 0; }
  void setInitialValue(const std::array<double, N>& initial) {
    for (std::size_t i = 0; i < N; i++) {
      para_[i] = initial[i];
    }
  }
  void setPreRotation(const Eigen::Matrix3d& rot) { pre_rotation_ = rot; }
  Eigen::Matrix<double, N, 1> getResult() {
    return Eigen::Map<Eigen::Matrix<double, N, 1>>(para_, N);
  }

 protected:
  const int DIM = N;
  double para_[N];
  Eigen::Matrix3d pre_rotation_ = Eigen::Matrix3d::Identity();
};

template <int N>
class CeresSolver : public Solver<N> {
 public:
  CeresSolver() = default;
  virtual ~CeresSolver(){};

  virtual bool solve(ceres::Solver::Options op = ceres::Solver::Options()) {
    op.num_threads = 4;
    op.linear_solver_type = ceres::DENSE_QR;
    op.minimizer_progress_to_stdout = false;
    ceres::Solve(op, &problem_, &summary);
    return summary.IsSolutionUsable();
  }

  virtual void getResidual(double final_cost, std::vector<double>* residuals) {
    problem_.Evaluate(ceres::Problem::EvaluateOptions(), &final_cost, residuals,
                      nullptr, nullptr);
  }

  virtual int getNumResiduals() { return problem_.NumResiduals(); }

  virtual ceres::Solver::Summary getSummary() { return summary; }

 protected:
  ceres::Solver::Summary summary;
  ceres::Problem problem_;
  ceres::LossFunction* loss_function_;
};

class SolvePoseFromPointToPlaneCeres : public CeresSolver<7> {
 public:
  SolvePoseFromPointToPlaneCeres() {
    para_[0] = 0;  // x
    para_[1] = 0;  // y
    para_[2] = 0;  // z
    para_[3] = 0;  // q_x
    para_[4] = 0;  // q_y
    para_[5] = 0;  // q_z
    para_[6] = 1;  // q_w

    para_t_ = para_;
    para_q_ = &para_[3];
    loss_function_ = new ceres::HuberLoss(0.1);

    ceres::Manifold* m = new ceres::EigenQuaternionManifold();
    problem_.AddParameterBlock(para_q_, 4, m);
    problem_.AddParameterBlock(para_t_, 3);
  }

  virtual void addData(const Target& measured_target,
                       const Target& ref_target) override {
    auto* cost_func = CostFunctor::create(measured_target.pos,
                                          ref_target.normal, ref_target.pos);
    problem_.AddResidualBlock(cost_func, loss_function_, para_q_, para_t_);
  }

  virtual bool solve(
      ceres::Solver::Options op = ceres::Solver::Options()) override {
    ceres::Solve(op, &problem_, &summary);
    if (summary.IsSolutionUsable()) {
      // std::cout << summary.BriefReport() << std::endl;
      return true;
    }
    return false;
  }
  ceres::Solver::Summary getSummary() { return summary; }

 private:
  double* para_t_;
  double* para_q_;
  class CostFunctor {
   public:
    CostFunctor(const Eigen::Vector3d& curr_p,
                const Eigen::Vector3d& target_norm,
                const Eigen::Vector3d& target_center)
        : cur_p_(curr_p),
          target_norm_(target_norm),
          target_center_(target_center) {}

    template <typename T>
    bool operator()(const T* para_q, const T* para_t, T* residual) const {
      Eigen::Matrix<T, 3, 1> plane_point(
          T(target_center_.x()), T(target_center_.y()), T(target_center_.z()));
      Eigen::Matrix<T, 3, 1> cur_point(T(cur_p_.x()), T(cur_p_.y()),
                                       T(cur_p_.z()));
      Eigen::Matrix<T, 3, 1> plane_normal(
          T(target_norm_.x()), T(target_norm_.y()), T(target_norm_.z()));

      Eigen::Matrix<T, 3, 1> trans(para_t[0], para_t[1], para_t[2]);
      Eigen::Quaternion<T> q{para_q[3], para_q[0], para_q[1], para_q[2]};
      Eigen::Matrix<T, 3, 1> trans_p = q * cur_point + trans;

      residual[0] = (trans_p - plane_point).dot(plane_normal);
      return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& curr_norm,
                                       const Eigen::Vector3d& target_norm,
                                       const Eigen::Vector3d& center) {
      return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 4, 3>(
          new CostFunctor(curr_norm, target_norm, center)));
    }

   private:
    Eigen::Vector3d cur_p_;
    Eigen::Vector3d target_norm_;
    Eigen::Vector3d target_center_;
  };
};

class SolvePoseFromPointToPlaneCeresSophus : public CeresSolver<7> {
 public:
  SolvePoseFromPointToPlaneCeresSophus() {
    para_[0] = 0;  // x
    para_[1] = 0;  // y
    para_[2] = 0;  // z
    para_[3] = 0;  // q_x
    para_[4] = 0;  // q_y
    para_[5] = 0;  // q_z
    para_[6] = 1;  // q_w
    for (std::size_t i = 0; i < 3; ++i) {
      para_t_[i] = 0;
      para_w_[i] = 0;
    }

    loss_function_ = new ceres::HuberLoss(0.1);
    problem_.AddParameterBlock(para_t_, 3);
    problem_.AddParameterBlock(para_w_, 3);
  }

  virtual void addData(const Target& measured_target,
                       const Target& ref_target) override {
    auto* cost_func =
        new CostFunctor(measured_target.pos, ref_target.normal, ref_target.pos);
    problem_.AddResidualBlock(cost_func, loss_function_, para_w_, para_t_);
  }

  virtual bool solve(
      ceres::Solver::Options op = ceres::Solver::Options()) override {
    ceres::Solve(op, &problem_, &summary);
    if (summary.IsSolutionUsable()) {
      // std::cout << summary.BriefReport() << std::endl;

      Eigen::Vector3d so_w;
      so_w << para_w_[0], para_w_[1], para_w_[2];
      Sophus::SO3d so3 = Sophus::SO3d::exp(so_w);
      para_[0] = para_t_[0];
      para_[1] = para_t_[1];
      para_[2] = para_t_[2];
      auto so3_unit_quaternion = so3.unit_quaternion();
      para_[3] = so3_unit_quaternion.x();
      para_[4] = so3_unit_quaternion.y();
      para_[5] = so3_unit_quaternion.z();
      para_[6] = so3_unit_quaternion.w();
      return true;
    }
    return false;
  }
  ceres::Solver::Summary getSummary() { return summary; }

 private:
  double para_t_[3];
  double para_w_[3];

  class CostFunctor : public ceres::SizedCostFunction<1, 3, 3> {
   public:
    CostFunctor(const Eigen::Vector3d& curr_p,
                const Eigen::Vector3d& target_normal,
                const Eigen::Vector3d& target_center)
        : cur_p_(curr_p),
          target_normal_(target_normal),
          target_center_(target_center) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
      double const* param_w = parameters[0];
      double const* param_t = parameters[1];
      Eigen::Vector3d trans(param_t[0], param_t[1], param_t[2]);
      Eigen::Vector3d so_w;
      so_w << param_w[0], param_w[1], param_w[2];
      Eigen::Vector3d q_p = Sophus::SO3d::exp(so_w).matrix() * cur_p_;
      residuals[0] = (q_p + trans - target_center_).dot(target_normal_);

      if (jacobians) {
        if (jacobians[0]) {  // param_w, left disturbance
          Eigen::Vector3d jacob = q_p.cross(target_normal_);
          jacobians[0][0] = jacob[0];
          jacobians[0][1] = jacob[1];
          jacobians[0][2] = jacob[2];
        }

        if (jacobians[1]) {  // param_t
          jacobians[1][0] = target_normal_[0];
          jacobians[1][1] = target_normal_[1];
          jacobians[1][2] = target_normal_[2];
        }
      }
      return true;
    }

   private:
    Eigen::Vector3d cur_p_;
    Eigen::Vector3d target_normal_;
    Eigen::Vector3d target_center_;
  };
};
