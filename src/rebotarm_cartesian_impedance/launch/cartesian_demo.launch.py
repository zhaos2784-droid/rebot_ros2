"""原 MoveIt demo 加一个待命的笛卡尔阻抗控制器；不改原 demo.launch.py。

RViz 到位后，在另一个终端手动切换：
ros2 control switch_controllers --deactivate arm_controller --activate cartesian_impedance_controller --strict
切回轨迹控制：
ros2 control switch_controllers --deactivate cartesian_impedance_controller --activate arm_controller --strict
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('rebotarm_moveit_config'))
    controller_share = Path(get_package_share_directory('rebotarm_cartesian_impedance'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit_share / 'launch/demo.launch.py'))),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[
                'cartesian_impedance_controller', '--controller-manager', '/controller_manager',
                '--controller-type', 'rebotarm_cartesian_impedance/CartesianController',
                '--param-file', str(controller_share / 'config/cartesian_controller.yaml'),
                '--inactive',
            ],
        ),
    ])
