#include "rebotarm_hardware/gravity_compensation.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rebotarm_hardware
{

struct GravityCompensation::Impl
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd q;
  std::array<int, 6> q_index{};
  std::array<int, 6> v_index{};
  Vector scale{};
};

GravityCompensation::GravityCompensation() = default;
GravityCompensation::~GravityCompensation() = default;

void GravityCompensation::configure(
  const std::string & urdf_xml, const std::array<double, 3> & gravity,
  const Vector & torque_scale)
{
  auto next = std::make_unique<Impl>();
  pinocchio::Model full_model;
  // 直接使用 ros2_control 收到的完整 robot_description，避免加载另一份 URDF。
  pinocchio::urdf::buildModelFromXML(urdf_xml, full_model);
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex id = 1;
    id < static_cast<pinocchio::JointIndex>(full_model.njoints); ++id)
  {
    bool arm_joint = false;
    for (int i = 1; i <= 6; ++i) {
      arm_joint |= full_model.names[id] == "joint" + std::to_string(i);
    }
    if (!arm_joint) {
      locked.push_back(id);
    }
  }
  // 夹爪电机角度尚未标定成手指位移：手指固定在模型中立位置。
  // 保留夹爪/手指的质量；不把第七电机角度错误地当作手指的米制位移。
  pinocchio::buildReducedModel(full_model, locked, pinocchio::neutral(full_model), next->model);
  if (next->model.nq != 6 || next->model.nv != 6) {
    throw std::runtime_error("Gravity model must contain six scalar arm joints");
  }
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(gravity[i])) {
      throw std::invalid_argument("Non-finite gravity vector");
    }
    next->model.gravity.linear()[i] = gravity[i];
  }
  next->model.gravity.angular().setZero();
  for (int i = 0; i < 6; ++i) {
    const auto name = "joint" + std::to_string(i + 1);
    if (!next->model.existJointName(name)) {
      throw std::runtime_error("Missing gravity joint: " + name);
    }
    const auto & joint = next->model.joints[next->model.getJointId(name)];
    if (joint.nq() != 1 || joint.nv() != 1 || !std::isfinite(torque_scale[i]) ||
      torque_scale[i] < 0.0 || torque_scale[i] > 2.0)
    {
      throw std::invalid_argument("Invalid gravity mapping/scale: " + name);
    }
    next->q_index[i] = joint.idx_q();
    next->v_index[i] = joint.idx_v();
  }
  next->scale = torque_scale;
  next->q = pinocchio::neutral(next->model);
  next->data = std::make_unique<pinocchio::Data>(next->model);
  impl_ = std::move(next);
}

GravityCompensation::Vector GravityCompensation::compute(const Vector & actual_positions)
{
  if (!impl_) {
    throw std::runtime_error("Gravity model is not configured");
  }
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(actual_positions[i])) {
      throw std::invalid_argument("Non-finite actual gravity position");
    }
    impl_->q[impl_->q_index[i]] = actual_positions[i];
  }
  // 模型和工作区只在 configure 创建；控制周期内只更新 q、重算 g(q)。
  const auto & g = pinocchio::computeGeneralizedGravity(impl_->model, *impl_->data, impl_->q);
  Vector result{};
  for (int i = 0; i < 6; ++i) {
    result[i] = g[impl_->v_index[i]] * impl_->scale[i];
    if (!std::isfinite(result[i])) {
      throw std::runtime_error("Non-finite gravity torque");
    }
  }
  return result;
}

}  // namespace rebotarm_hardware
