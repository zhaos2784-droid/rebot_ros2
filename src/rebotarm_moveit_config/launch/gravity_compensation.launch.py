"""独立重力补偿：复用 C++ 硬件接口，不启动 MoveIt 或轨迹控制器。

六轴 Kp=0，目标速度=0，沿用 URDF 的 Kd、重力缩放和力矩限幅。
只在本次启动的内存模型中覆盖 Kp，不修改原 URDF 或普通 demo。
此模式不恢复到初始位置；补偿误差可能导致漂移，测试时需托住机械臂。
Ctrl+C 正常停用硬件时失能，不执行回零，退出前必须可靠支撑。
"""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch(context):
    description_share = Path(get_package_share_directory('my_rebot_description'))
    root = ET.parse(description_share / 'urdf/ReBot_Arm_DM.urdf').getroot()
    hardware = root.find('ros2_control')
    # 启动检查失败时不打开串口，更不会使能电机。
    if hardware is None:
        raise RuntimeError('URDF 缺少 ros2_control 配置')
    for i in range(1, 7):
        joint = hardware.find(f"joint[@name='joint{i}']")
        if joint is None or joint.find("param[@name='control_mode']").text != 'mit':
            raise RuntimeError(f'joint{i} 必须配置 MIT 模式')
        joint.find("param[@name='mit_kp']").text = '0.0'

    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    if mock:
        hardware.find('hardware/plugin').text = 'mock_components/GenericSystem'
    else:
        port = hardware.find("hardware/param[@name='port_name']").text
        busy = subprocess.run(['fuser', port], capture_output=True, check=False)
        if busy.returncode == 0:
            raise RuntimeError(f'{port} 正被其他进程占用；请支撑机械臂并关闭原 demo/网关后再启动')

    config_share = Path(get_package_share_directory('rebotarm_moveit_config'))
    description = ParameterValue(ET.tostring(root, encoding='unicode'), value_type=str)
    return [
        LogInfo(msg='独立重力补偿：六轴 Kp=0，保留 Kd 和力矩限幅；不会回原位。退出前支撑机械臂。'),
        Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            parameters=[{'robot_description': description}, str(config_share / 'config/ros2_controllers.yaml')],
        ),
        # 只加载反馈广播器，不加载 arm_controller，也不启动 RViz/MoveIt。
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false', description='true 仅用于离线模拟检查'),
        OpaqueFunction(function=_launch),
    ])
