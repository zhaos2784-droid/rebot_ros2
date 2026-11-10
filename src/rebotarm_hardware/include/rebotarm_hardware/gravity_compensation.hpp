#pragma once

#include <array>
#include <memory>
#include <string>

namespace rebotarm_hardware
{

// 只做模型和重力计算，不连接电机，不管理使能/退出。
// 输入/输出均为 URDF 关节坐标：rad / N*m，按 joint1..joint6 排列。
class GravityCompensation
{
public:
  using Vector = std::array<double, 6>;
  GravityCompensation();
  ~GravityCompensation();
  void configure(
    const std::string & urdf_xml,
    const std::array<double, 3> & gravity,
    const Vector & torque_scale);
  Vector compute(const Vector & actual_positions);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rebotarm_hardware
