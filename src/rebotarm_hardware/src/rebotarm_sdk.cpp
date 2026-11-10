#include "rebotarm_hardware/rebotarm_sdk.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>

#include "motorbridge/motorbridge.hpp"

namespace rebotarm_hardware
{

using Clock = std::chrono::steady_clock;

struct RebotArmSdk::Impl
{
  std::array<MotorConfig, joint_count> config;
  std::unique_ptr<motorbridge::Controller> controller;
  std::vector<motorbridge::Motor> motors;
  std::size_t next_feedback_joint{};
  Clock::time_point next_feedback_request{};
  Clock::time_point next_write{};
};

RebotArmSdk::RebotArmSdk() = default;
RebotArmSdk::~RebotArmSdk() = default;

void RebotArmSdk::connect(
  const std::string & port, std::uint32_t baud,
  const std::array<MotorConfig, joint_count> & motors)
{
  auto next = std::make_unique<Impl>();
  next->config = motors;
  next->controller = std::make_unique<motorbridge::Controller>(
    motorbridge::Controller::from_dm_serial(port, baud));
  next->motors.reserve(joint_count);
  for (const auto & motor : motors) {
    next->motors.push_back(
      next->controller->add_damiao_motor(motor.motor_id, motor.feedback_id, motor.model));
  }
  impl_ = std::move(next);
}

std::array<RebotArmSdk::State, RebotArmSdk::joint_count> RebotArmSdk::read_initial_states()
{
  if (!impl_) {
    throw std::runtime_error("MotorBridge is not connected");
  }
  std::array<State, joint_count> states{};
  std::array<bool, joint_count> received{};
  for (auto & motor : impl_->motors) {
    motor.request_feedback();
  }
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  while (Clock::now() < deadline) {
    impl_->controller->poll_feedback_once();
    bool all_received = true;
    for (std::size_t i = 0; i < joint_count; ++i) {
      const auto state = impl_->motors[i].get_state();
      if (state && std::isfinite(state->pos) && std::isfinite(state->vel) &&
        std::isfinite(state->torq) && state->status_code <= 1)
      {
        states[i] = {state->pos, state->vel, state->torq};
        received[i] = true;
      }
      all_received &= received[i];
    }
    if (all_received) {
      return states;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("No valid initial feedback from all seven motors");
}

void RebotArmSdk::enable(const std::array<Command, joint_count> & initial_commands)
{
  if (!impl_) {
    throw std::runtime_error("MotorBridge is not connected");
  }
  try {
    // 初始目标和重力力矩已由硬件层按实测姿态准备好。
    // 在使能前切完模式，避免使能后又切模式造成短暂失能。
    for (std::size_t i = 0; i < joint_count; ++i) {
      if (impl_->config[i].mode != Mode::Mit) {
        throw std::runtime_error("This hardware configuration requires MIT motors");
      }
      impl_->motors[i].ensure_mode(motorbridge::Mode::MIT);
    }
    impl_->controller->enable_all();
    impl_->next_feedback_request = Clock::now();
    impl_->next_write = Clock::now();
    write_commands(initial_commands);
  } catch (...) {
    try {
      impl_->controller->disable_all();
    } catch (...) {
      // Preserve the original activation error.
    }
    throw;
  }
}

void RebotArmSdk::disable()
{
  if (!impl_) {
    return;
  }
  // inactive 仍保留串口对象，允许同一硬件实例重新激活。
  impl_->controller->disable_all();
}

void RebotArmSdk::close()
{
  impl_.reset();
}

void RebotArmSdk::read_states(std::array<State, joint_count> & states)
{
  if (!impl_) {
    throw std::runtime_error("MotorBridge is not connected");
  }
  impl_->controller->poll_feedback_once();
  const auto now = Clock::now();
  if (now >= impl_->next_feedback_request) {
    impl_->motors[impl_->next_feedback_joint].request_feedback();
    impl_->next_feedback_joint = (impl_->next_feedback_joint + 1) % joint_count;
    impl_->next_feedback_request = now + std::chrono::milliseconds(10);
  }
  for (std::size_t i = 0; i < joint_count; ++i) {
    const auto state = impl_->motors[i].get_state();
    if (state && std::isfinite(state->pos)) {
      states[i].position = state->pos;
      if (std::isfinite(state->vel)) {
        states[i].velocity = state->vel;
      }
      if (std::isfinite(state->torq)) {
        states[i].effort = state->torq;
      }
    }
  }
}

void RebotArmSdk::write_commands(const std::array<Command, joint_count> & commands)
{
  if (!impl_) {
    throw std::runtime_error("MotorBridge is not connected");
  }
  for (std::size_t i = 0; i < joint_count; ++i) {
    const auto & command = commands[i];
    if (!std::isfinite(command.position)) {
      throw std::invalid_argument("Non-finite arm command");
    }
    if (impl_->config[i].mode == Mode::Mit &&
      (!std::isfinite(command.velocity) || !std::isfinite(command.effort) ||
      !std::isfinite(command.kp) || !std::isfinite(command.kd)))
    {
      throw std::invalid_argument("Non-finite MIT command");
    }
  }
  const auto now = Clock::now();
  if (now < impl_->next_write) {
    return;
  }
  for (std::size_t i = 0; i < joint_count; ++i) {
    const auto & command = commands[i];
    if (impl_->config[i].mode == Mode::Mit) {
      impl_->motors[i].send_mit(
        static_cast<float>(command.position), static_cast<float>(command.velocity),
        static_cast<float>(command.kp), static_cast<float>(command.kd),
        static_cast<float>(command.effort));
    } else {
      impl_->motors[i].send_pos_vel(
        static_cast<float>(command.position), impl_->config[i].velocity_limit);
    }
  }
  impl_->next_write = now + std::chrono::milliseconds(5);
}

}  // namespace rebotarm_hardware
