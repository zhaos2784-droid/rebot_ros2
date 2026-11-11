#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rebotarm_hardware/rebotarm_sdk.hpp"
#include "rebotarm_hardware/gravity_compensation.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rebotarm_hardware
{

class RebotArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t joint_count_ = RebotArmSdk::joint_count;

  std::string port_name_;
  std::uint32_t baud_{921600};
  std::array<RebotArmSdk::MotorConfig, joint_count_> config_{};
  std::array<RebotArmSdk::State, joint_count_> states_{};
  RebotArmSdk::State finger_state_{};  // 第七电机反馈换算后的手指位移/速度/广义力。
  std::array<RebotArmSdk::Command, joint_count_> commands_{};
  RebotArmSdk sdk_;
  GravityCompensation gravity_;
  GravityCompensation::Vector gravity_scale_{};
  std::array<double, 3> gravity_vector_{0.0, 0.0, -9.81};
  std::array<double, joint_count_> torque_limit_{};
  std::array<double, joint_count_> position_lower_{};
  std::array<double, joint_count_> position_upper_{};
  std::array<RebotArmSdk::Command, joint_count_> output_{};
  bool active_{false};
};

}  // namespace rebotarm_hardware
