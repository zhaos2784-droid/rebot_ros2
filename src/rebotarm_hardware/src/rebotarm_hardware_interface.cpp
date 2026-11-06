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

namespace rebotarm_hardware
{

namespace
{
const auto logger = rclcpp::get_logger("rebotarm_hardware");

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
    for (std::size_t i = 0; i < joint_count_; ++i) {
      const auto & joint = info_.joints[i];
      const auto expected_name = i < 6 ? "joint" + std::to_string(i + 1) : "gripper";
      if (joint.name != expected_name) {
        throw std::invalid_argument("Expected joint1..joint6 followed by gripper");
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
    }
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
    states_ = sdk_.enable();
    for (std::size_t i = 0; i < joint_count_; ++i) {
      commands_[i].position = states_[i].position;
      commands_[i].velocity = 0.0;
      commands_[i].effort = 0.0;
      commands_[i].kp = config_[i].mit_kp;
      commands_[i].kd = config_[i].mit_kd;
    }
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
    result.emplace_back(name, hardware_interface::HW_IF_POSITION, &states_[i].position);
    result.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &states_[i].velocity);
    result.emplace_back(name, hardware_interface::HW_IF_EFFORT, &states_[i].effort);
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
  try {
    sdk_.read_states(states_);
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor read failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type RebotArmHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    sdk_.write_commands(commands_);
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Motor write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace rebotarm_hardware

PLUGINLIB_EXPORT_CLASS(
  rebotarm_hardware::RebotArmHardwareInterface, hardware_interface::SystemInterface)
