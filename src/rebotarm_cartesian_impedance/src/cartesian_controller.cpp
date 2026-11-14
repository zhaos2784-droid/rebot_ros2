#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rebotarm_cartesian_impedance
{

class CartesianController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<double>("translation_k", 80.0);     // N/m
    auto_declare<double>("translation_d", 8.0);      // N s/m
    auto_declare<double>("rotation_k", 2.0);        // N m/rad
    auto_declare<double>("rotation_d", 0.2);        // N m s/rad
    // 关节 1 的弱回中弹簧：纯 6 轴笛卡尔阻抗没有零空间，只能小增益折中。
    auto_declare<double>("joint_k", 3.0);           // N m/rad
    auto_declare<double>("joint_d", 0.3);           // N m s/rad
    auto_declare<std::string>("frame", "world");   // world or tcp
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    std::vector<std::string> names;
    names.reserve(30);
    for (int i = 1; i <= 6; ++i) {
      const auto joint = "joint" + std::to_string(i) + "/";
      names.insert(names.end(), {
        joint + "position", joint + "velocity", joint + "effort",
        joint + "kp", joint + "kd"});
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> names;
    names.reserve(12);
    for (int i = 1; i <= 6; ++i) {
      const auto joint = "joint" + std::to_string(i) + "/";
      names.insert(names.end(), {joint + "position", joint + "velocity"});
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      k_pos_ = get_node()->get_parameter("translation_k").as_double();
      d_pos_ = get_node()->get_parameter("translation_d").as_double();
      k_rot_ = get_node()->get_parameter("rotation_k").as_double();
      d_rot_ = get_node()->get_parameter("rotation_d").as_double();
      kp_joint_ = get_node()->get_parameter("joint_k").as_double();
      kd_joint_ = get_node()->get_parameter("joint_d").as_double();
      const auto frame = get_node()->get_parameter("frame").as_string();
      if (frame != "world" && frame != "tcp") {
        throw std::runtime_error("frame must be world or tcp");
      }
      tcp_frame_ = frame == "tcp";
      const auto path = ament_index_cpp::get_package_share_directory("my_rebot_description") +
        "/urdf/ReBot_Arm_DM.urdf";
      pinocchio::urdf::buildModel(path, model_);
      frame_id_ = model_.getFrameId("end_link");
      if (frame_id_ == model_.nframes) {
        throw std::runtime_error("URDF has no end_link");
      }
      for (int i = 0; i < 6; ++i) {
        const auto joint_id = model_.getJointId("joint" + std::to_string(i + 1));
        if (joint_id == model_.njoints || model_.joints[joint_id].nq() != 1 ||
          model_.joints[joint_id].nv() != 1)
        {
          throw std::runtime_error("Missing scalar arm joint in URDF");
        }
        q_index_[i] = model_.joints[joint_id].idx_q();
        v_index_[i] = model_.joints[joint_id].idx_v();
      }
      data_ = std::make_unique<pinocchio::Data>(model_);
      q_ = pinocchio::neutral(model_);
      dq_ = Eigen::VectorXd::Zero(model_.nv);
      jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv);
      tau_ = Eigen::VectorXd::Zero(model_.nv);
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cartesian controller configure: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!read_state()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    for (int i = 0; i < 6; ++i) {
      q_home_[i] = q_[q_index_[i]];   // 回中目标 = 切换时的关节角
    }
    pinocchio::forwardKinematics(model_, *data_, q_);
    pinocchio::updateFramePlacements(model_, *data_);
    target_ = data_->oMf[frame_id_];  // 当前 TCP 就是回位目标，切换时不跳变。
    for (int i = 0; i < 6; ++i) {
      previous_kp_[i] = command_interfaces_[5 * i + 3].get_value();
      previous_kd_[i] = command_interfaces_[5 * i + 4].get_value();
    }
    write_command(Eigen::VectorXd::Zero(model_.nv), 0.0, 0.8);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!read_state()) {
      return controller_interface::return_type::ERROR;
    }
    pinocchio::computeJointJacobians(model_, *data_, q_);
    pinocchio::updateFramePlacements(model_, *data_);
    jacobian_.setZero();
    pinocchio::getFrameJacobian(
      model_, *data_, frame_id_,
      tcp_frame_ ? pinocchio::LOCAL : pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

    const auto & current = data_->oMf[frame_id_];
    Eigen::Matrix<double, 6, 1> error;
    if (tcp_frame_) {
      error.head<3>() = current.rotation().transpose() *
        (target_.translation() - current.translation());
      error.tail<3>() = pinocchio::log3(current.rotation().transpose() * target_.rotation());
    } else {
      error.head<3>() = target_.translation() - current.translation();
      error.tail<3>() = pinocchio::log3(target_.rotation() * current.rotation().transpose());
    }
    const Eigen::Matrix<double, 6, 1> velocity = jacobian_ * dq_;
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.head<3>() = k_pos_ * error.head<3>() - d_pos_ * velocity.head<3>();
    wrench.tail<3>() = k_rot_ * error.tail<3>() - d_rot_ * velocity.tail<3>();
    tau_.noalias() = jacobian_.transpose() * wrench;
    // joint1 弱回中弹簧：纯 6 轴笛卡尔阻抗没有零空间，只能小增益折中；
    // 只加第一个关节，避免整条臂变硬、更难拖动。
    {
      const int v1 = v_index_[0];
      tau_[v1] += kp_joint_ * (q_home_[0] - q_[q_index_[0]]) - kd_joint_ * dq_[v1];
    }
    if (!tau_.allFinite()) {
      return controller_interface::return_type::ERROR;
    }
    // effort 只包含笛卡尔回位力矩；现有硬件接口负责另加一次重力补偿。
    // 保留一个很小的电机本地 Kd（kHz 级、用编码器反馈），用来压住
    // 主机 100 Hz 环路压不住的高频抖动；它不改变笛卡尔刚度。
    write_command(tau_, 0.0, 0.8);
    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // 归还原增益，后续切回 MoveIt 轨迹控制器仍可正常运动。
    write_command(Eigen::VectorXd::Zero(model_.nv), 0.0, 0.0);
    for (int i = 0; i < 6; ++i) {
      command_interfaces_[5 * i + 3].set_value(previous_kp_[i]);
      command_interfaces_[5 * i + 4].set_value(previous_kd_[i]);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

private:
  bool read_state()
  {
    for (int i = 0; i < 6; ++i) {
      const double q = state_interfaces_[2 * i].get_value();
      const double dq = state_interfaces_[2 * i + 1].get_value();
      if (!std::isfinite(q) || !std::isfinite(dq)) {
        return false;
      }
      q_[q_index_[i]] = q;
      dq_[v_index_[i]] = dq;
    }
    return true;
  }

  void write_command(const Eigen::VectorXd & effort, double kp, double kd)
  {
    for (int i = 0; i < 6; ++i) {
      command_interfaces_[5 * i].set_value(q_[q_index_[i]]);
      command_interfaces_[5 * i + 1].set_value(0.0);
      command_interfaces_[5 * i + 2].set_value(effort[v_index_[i]]);
      command_interfaces_[5 * i + 3].set_value(kp);
      command_interfaces_[5 * i + 4].set_value(kd);
    }
  }

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex frame_id_{};
  pinocchio::SE3 target_;
  Eigen::VectorXd q_, dq_, tau_;
  Eigen::MatrixXd jacobian_;
  std::array<int, 6> q_index_{}, v_index_{};
  std::array<double, 6> previous_kp_{}, previous_kd_{};
  std::array<double, 6> q_home_{};
  double k_pos_{}, d_pos_{}, k_rot_{}, d_rot_{};
  double kp_joint_{}, kd_joint_{};
  bool tcp_frame_{};
};

}  // namespace rebotarm_cartesian_impedance

PLUGINLIB_EXPORT_CLASS(
  rebotarm_cartesian_impedance::CartesianController, controller_interface::ControllerInterface)
