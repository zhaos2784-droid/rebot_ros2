#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rebotarm_hardware
{

// C++ wrapper around MotorBridge for six arm motors and one gripper motor.
class RebotArmSdk
{
public:
  static constexpr std::size_t joint_count = 7;
  enum class Mode { PosVel, Mit };

  struct MotorConfig
  {
    std::uint16_t motor_id{};
    std::uint16_t feedback_id{};
    std::string model;
    float velocity_limit{};
    float vel_kp{};
    float vel_ki{};
    float pos_kp{};
    float pos_ki{};
    Mode mode{Mode::PosVel};
    float mit_kp{};
    float mit_kd{};
  };

  struct State
  {
    double position{};
    double velocity{};
    double effort{};
  };

  struct Command
  {
    double position{};
    double velocity{};
    double effort{};
    double kp{};
    double kd{};
  };

  RebotArmSdk();
  ~RebotArmSdk();
  RebotArmSdk(const RebotArmSdk &) = delete;
  RebotArmSdk & operator=(const RebotArmSdk &) = delete;

  void connect(
    const std::string & port, std::uint32_t baud,
    const std::array<MotorConfig, joint_count> & motors);
  std::array<State, joint_count> enable();
  void disable();
  void close();
  void read_states(std::array<State, joint_count> & states);
  void write_commands(const std::array<Command, joint_count> & commands);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rebotarm_hardware
