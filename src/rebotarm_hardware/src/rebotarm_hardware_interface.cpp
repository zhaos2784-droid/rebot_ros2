#include "rebotarm_hardware/rebotarm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace rebotarm_hardware
{

namespace
{
const auto logger = rclcpp::get_logger("rebotarm_hardware");

// 单侧手指 0..0.025 m 对应电机 0..-5 rad；沿用厂家 DM 开闭端点约定。
// MIT 的 Kp/Kd、力矩限幅仍使用原电机单位，不在这里改动。
constexpr double gripper_rad_per_meter = -5.0 / 0.025;

RebotArmSdk::State finger_feedback(const RebotArmSdk::State & motor)
{
  return {motor.position / gripper_rad_per_meter,
    motor.velocity / gripper_rad_per_meter, motor.effort * gripper_rad_per_meter};
}

const std::string & required(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key)
{
  const auto it = parameters.find(key);
  if (it == parameters.end() || it->second.empty()) {
    throw std::runtime_error("Missing parameter: " + key);
  }
  return it->second;
}

std::uint16_t parse_id(const std::string & value)
{
  const auto parsed = std::stoul(value, nullptr, 0);
  if (parsed > std::numeric_limits<std::uint16_t>::max()) {
    throw std::out_of_range("Motor ID exceeds uint16");
  }
  return static_cast<std::uint16_t>(parsed);
}

float parse_float(const std::string & value)
{
  const auto parsed = std::stof(value);
  if (!std::isfinite(parsed)) {
    throw std::invalid_argument("Non-finite motor parameter");
  }
  return parsed;
}

template<typename Interfaces>
bool has_interface(const Interfaces & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&](const auto & interface) {return interface.name == name;});
}
}  // namespace

hardware_interface::CallbackReturn RebotArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != joint_count_) {
    RCLCPP_ERROR(logger, "Expected six arm joints and one gripper motor; got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    port_name_ = required(info_.hardware_parameters, "port_name");
    baud_ = std::stoul(required(info_.hardware_parameters, "baud"));
    if (baud_ == 0) {
      throw std::invalid_argument("baud must be positive");
    }
    urdf::Model limits_model;
    if (!limits_model.initString(info_.original_xml)) {
      throw std::runtime_error("Cannot parse robot_description for joint limits");
    }
    for (int axis = 0; axis < 3; ++axis) {
      const std::string key = std::string("gravity_") + "xyz"[axis];
      const auto it = info_.hardware_parameters.find(key);
      if (it != info_.hardware_parameters.end()) {
        gravity_vector_[axis] = parse_float(it->second);
      }
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
      const auto & joint = info_.joints[i];
      const auto expected_name = i < 6 ? "joint" + std::to_string(i + 1) : "finger_left";
      if (joint.name != expected_name) {
        throw std::invalid_argument("Expected joint1..joint6 followed by finger_left");
      }
      auto & c = config_[i];
      const auto & mode = required(joint.parameters, "control_mode");
      if (mode == "mit") {
        c.mode = RebotArmSdk::Mode::Mit;
      } else if (mode == "pos_vel") {
        c.mode = RebotArmSdk::Mode::PosVel;
      } else {
        throw std::invalid_argument(joint.name + ": control_mode must be mit or pos_vel");
      }
      const auto command_count = c.mode == RebotArmSdk::Mode::Mit ? 5U : 1U;
      if (joint.command_interfaces.size() != command_count ||
        !has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) ||
        (c.mode == RebotArmSdk::Mode::Mit &&
        (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !has_interface(joint.command_interfaces, hardware_interface::HW_IF_EFFORT) ||
        !has_interface(joint.command_interfaces, "kp") ||
        !has_interface(joint.command_interfaces, "kd"))))
      {
        throw std::invalid_argument(joint.name + ": invalid command interfaces for " + mode);
      }
      if (joint.state_interfaces.size() != 3 ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT))
      {
        throw std::invalid_argument(joint.name + ": expected position, velocity, effort states");
      }
      c.motor_id = parse_id(required(joint.parameters, "motor_id"));
      c.feedback_id = parse_id(required(joint.parameters, "feedback_id"));
      c.model = required(joint.parameters, "model");
      c.velocity_limit = parse_float(required(joint.parameters, "velocity_limit"));
      c.vel_kp = parse_float(required(joint.parameters, "vel_kp"));
      c.vel_ki = parse_float(required(joint.parameters, "vel_ki"));
      c.pos_kp = parse_float(required(joint.parameters, "pos_kp"));
      c.pos_ki = parse_float(required(joint.parameters, "pos_ki"));
      c.mit_kp = parse_float(required(joint.parameters, "mit_kp"));
      c.mit_kd = parse_float(required(joint.parameters, "mit_kd"));
      if (c.velocity_limit <= 0) {
        throw std::invalid_argument(joint.name + ": velocity_limit must be positive");
      }
      if (c.mit_kp < 0 || c.mit_kd < 0) {
        throw std::invalid_argument(joint.name + ": MIT gains must be nonnegative");
      }
      const auto joint_model = limits_model.getJoint(joint.name);
      // 第七电机对外使用米制 finger_left；以下限幅仍在电机弧度域执行。
      if (i < 6) {
        if (!joint_model || !joint_model->limits || c.mode != RebotArmSdk::Mode::Mit) {
          throw std::invalid_argument(joint.name + ": arm requires MIT and URDF joint limits");
        }
        position_lower_[i] = joint_model->limits->lower;
        position_upper_[i] = joint_model->limits->upper;
        torque_limit_[i] = std::min(
          static_cast<double>(parse_float(required(joint.parameters, "torque_limit"))),
          joint_model->limits->effort);
        gravity_scale_[i] = parse_float(required(joint.parameters, "gravity_scale"));
      } else {
        position_lower_[i] = -5.0;
        position_upper_[i] = 0.0;
        torque_limit_[i] = parse_float(required(joint.parameters, "torque_limit"));
      }
      if (torque_limit_[i] <= 0 || c.mit_kp > 500 || c.mit_kd > 5) {
        throw std::invalid_argument(joint.name + ": invalid torque limit or MIT gain range");
      }
    }
    gravity_.configure(info_.original_xml, gravity_vector_, gravity_scale_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Invalid hardware configuration: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < joint_count_; ++i) {
    states_[i].position = std::numeric_limits<double>::quiet_NaN();
    states_[i].velocity = std::numeric_limits<double>::quiet_NaN();
    states_[i].effort = std::numeric_limits<double>::quiet_NaN();
    commands_[i].position = std::numeric_limits<double>::quiet_NaN();
    commands_[i].velocity = 0.0;
    commands_[i].effort = 0.0;
    commands_[i].kp = config_[i].mit_kp;
    commands_[i].kd = config_[i].mit_kd;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RebotArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    sdk_.connect(port_name_, baud_, config_);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "MotorBridge setup failed: %s", e.what());
    sdk_.close();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn RebotArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  try {
    states_ = sdk_.read_initial_states();
    finger_state_ = finger_feedback(states_[6]);
    for (std::size_t i = 0; i < joint_count_; ++i) {
      commands_[i].position = i < 6 ? states_[i].position : finger_state_.position;
      commands_[i].velocity = 0.0;
      commands_[i].effort = 0.0;
      commands_[i].kp = config_[i].mit_kp;
      commands_[i].kd = config_[i].mit_kd;
    }
    GravityCompensation::Vector q{};
    for (std::size_t i = 0; i < 6; ++i) {
      q[i] = states_[i].position;
    }
    const auto gravity = gravity_.compute(q);
    output_ = commands_;
    for (std::size_t i = 0; i < 6; ++i) {
      output_[i].effort = std::clamp(gravity[i], -torque_limit_[i], torque_limit_[i]);
    }
    // 第一帧已经包含重力补偿，不在低刚度使能后等待若干周期才开始托举。
    output_[6].position *= gripper_rad_per_meter;  // 使能时仍保持当前电机位置。
    sdk_.enable(output_);
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor activation failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn RebotArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  try {
    sdk_.disable();
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor disable failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface>
RebotArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(3 * joint_count_);
  for (std::size_t i = 0; i < joint_count_; ++i) {
    const auto & name = info_.joints[i].name;
    auto & state = i < 6 ? states_[i] : finger_state_;
    result.emplace_back(name, hardware_interface::HW_IF_POSITION, &state.position);
    result.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state.velocity);
    result.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state.effort);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface>
RebotArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(5 * joint_count_);
  for (std::size_t i = 0; i < joint_count_; ++i) {
    const auto & name = info_.joints[i].name;
    result.emplace_back(name, hardware_interface::HW_IF_POSITION, &commands_[i].position);
    if (config_[i].mode == RebotArmSdk::Mode::Mit) {
      result.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &commands_[i].velocity);
      result.emplace_back(name, hardware_interface::HW_IF_EFFORT, &commands_[i].effort);
      result.emplace_back(name, "kp", &commands_[i].kp);
      result.emplace_back(name, "kd", &commands_[i].kd);
    }
  }
  return result;
}

hardware_interface::return_type RebotArmHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  try {
    sdk_.read_states(states_);
    finger_state_ = finger_feedback(states_[6]);
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor read failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type RebotArmHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  try {
    GravityCompensation::Vector q{};
    for (std::size_t i = 0; i < 6; ++i) {
      q[i] = states_[i].position;
    }
    const auto gravity = gravity_.compute(q);
    output_ = commands_;
    // 只在输出副本转换第七轴：命令缓冲保留米，SDK 和 PD 限幅使用弧度/力矩。
    output_[6].position *= gripper_rad_per_meter;
    output_[6].velocity *= gripper_rad_per_meter;
    output_[6].effort /= gripper_rad_per_meter;
    for (std::size_t i = 0; i < joint_count_; ++i) {
      auto & out = output_[i];
      const auto & state = states_[i];
      if (!std::isfinite(out.position) || !std::isfinite(out.velocity) ||
        !std::isfinite(out.effort) || !std::isfinite(out.kp) || !std::isfinite(out.kd) ||
        !std::isfinite(state.position) || !std::isfinite(state.velocity) ||
        out.kp < 0 || out.kp > 500 || out.kd < 0 || out.kd > 5)
      {
        throw std::runtime_error("Invalid MIT command/state");
      }
      out.position = std::clamp(out.position, position_lower_[i], position_upper_[i]);
      out.velocity = std::clamp(
        out.velocity, -static_cast<double>(config_[i].velocity_limit),
        static_cast<double>(config_[i].velocity_limit));
      // effort 命令表示“额外前馈”；重力在输出副本中逐周期叠加，不改写命令缓冲。
      out.effort += i < 6 ? gravity[i] : 0.0;
      // 限制包含 PD 在内的预计总力矩，而不只是限制重力前馈。
      // 电机用自己的高速反馈执行 PD；这里的限制不能代替驱动器硬件限流。
      const double pd = out.kp * (out.position - state.position) +
        out.kd * (out.velocity - state.velocity);
      if (std::abs(pd) > torque_limit_[i]) {
        const double gain_scale = torque_limit_[i] / std::abs(pd);
        out.kp *= gain_scale;
        out.kd *= gain_scale;
      }
      const double limited_pd = out.kp * (out.position - state.position) +
        out.kd * (out.velocity - state.velocity);
      out.effort = std::clamp(
        out.effort, -torque_limit_[i] - limited_pd, torque_limit_[i] - limited_pd);
    }
    sdk_.write_commands(output_);
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace rebotarm_hardware

PLUGINLIB_EXPORT_CLASS(
  rebotarm_hardware::RebotArmHardwareInterface, hardware_interface::SystemInterface)
