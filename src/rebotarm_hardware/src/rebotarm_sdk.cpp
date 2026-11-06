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

std::array<RebotArmSdk::State, RebotArmSdk::joint_count> RebotArmSdk::enable()
{
  if (!impl_) {
    throw std::runtime_error("MotorBridge is not connected");
  }

  std::array<State, joint_count> states{};
  std::array<bool, joint_count> received{};
  try {
    for (auto & motor : impl_->motors) {
      motor.request_feedback();
    }
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < deadline) {
      impl_->controller->poll_feedback_once();
      bool all_received = true;
      for (std::size_t i = 0; i < joint_count; ++i) {
        const auto state = impl_->motors[i].get_state();
        if (state && std::isfinite(state->pos)) {
          states[i] = {
            state->pos,
            std::isfinite(state->vel) ? state->vel : 0.0,
            std::isfinite(state->torq) ? state->torq : 0.0};
          received[i] = true;
        }
        all_received &= received[i];
      }
      if (all_received) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    for (bool got_feedback : received) {
      if (!got_feedback) {
        throw std::runtime_error("No initial feedback from all seven motors");
      }
    }

    impl_->controller->enable_all();
    for (std::size_t i = 0; i < joint_count; ++i) {
      auto & motor = impl_->motors[i];
      const auto & c = impl_->config[i];
      if (c.mode == Mode::Mit) {
        motor.ensure_mode(motorbridge::Mode::MIT);
        motor.send_mit(static_cast<float>(states[i].position), 0.0f, c.mit_kp, c.mit_kd, 0.0f);
      } else {
        motor.write_register_f32(25, c.vel_kp);
        motor.write_register_f32(26, c.vel_ki);
        motor.write_register_f32(27, c.pos_kp);
        motor.write_register_f32(28, c.pos_ki);
        motor.ensure_mode(motorbridge::Mode::POS_VEL);
        motor.send_pos_vel(static_cast<float>(states[i].position), c.velocity_limit);
      }
    }
    impl_->next_feedback_request = Clock::now();
    impl_->next_write = Clock::now() + std::chrono::milliseconds(5);
    return states;
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
  auto old = std::move(impl_);
  old->controller->disable_all();
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
