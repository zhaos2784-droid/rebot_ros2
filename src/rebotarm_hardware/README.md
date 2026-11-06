# rebotarm_hardware

ROS 2 Humble `ros2_control` hardware plugin for six Damiao arm joints and one gripper motor of reBotArm.
It uses MotorBridge 0.5.6 through the C++ wrapper and the ABI library installed by
the Python `motorbridge` package. The MotorBridge headers and license in this
package are copied from the official v0.5.6 release.

The C++ SDK wrapper is `include/rebotarm_hardware/rebotarm_sdk.hpp` and
`src/rebotarm_sdk.cpp`. It handles the serial bridge, motor feedback, enabling,
POS_VEL commands, and MIT commands. The ROS hardware interface is
`include/rebotarm_hardware/rebotarm_hardware_interface.hpp` and
`src/rebotarm_hardware_interface.cpp`. It maps ROS state and command interfaces
to the SDK.

The plugin name is `rebotarm_hardware/RebotArmHardwareInterface`. Its `ros2_control`
configuration lives in `my_rebot_description/urdf/ReBot_Arm_DM.urdf`; it needs
`/dev/ttyACM0` or the serial port named there. Activation reads all seven motor
positions before enabling and sending commands. Every joint exports position,
velocity, and effort state. `control_mode=pos_vel` exports a position command;
`control_mode=mit` exports position, velocity, effort (feedforward torque),
`kp`, and `kd` commands. The initial MIT velocity and effort commands are zero,
and gains come from the URDF. The default configuration uses POS_VEL on the
six arm joints and MIT on the gripper, matching the Python SDK. To use MIT on
an arm joint, change its `control_mode` to `mit` in the URDF and add the
velocity, effort, kp, and kd command interfaces shown on the gripper joint.
Reload the updated robot description and hardware component to apply a mode change.

The `gripper` interface uses raw motor angle in radians, as in the Python SDK.
The Python SDK sends motor position directly and returns raw motor position;
it has no motor-to-finger conversion. This interface does not drive the
`finger_left` prismatic URDF joint in meters. A measured scale, direction,
and zero offset are needed before connecting that joint to ros2_control.

Build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rebotarm_hardware my_rebot_description
source install/setup.bash
```

Building does not open the motor port. Loading and activating the hardware
through controller_manager does. Confirm motor IDs, baud, joint directions,
joint limits, and a safe pose before activating on a powered arm.
