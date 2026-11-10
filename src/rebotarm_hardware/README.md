# rebotarm_hardware

reBot B601-DM 的 C++ ros2_control 硬件接口，使用 MotorBridge ABI 控制六轴和夹爪。
七个电机均配置 MIT，每轴提供 position、velocity、effort、kp、kd 五个命令接口，
以及 position、velocity、effort 三个状态接口。

独立重力库在 include/rebotarm_hardware/gravity_compensation.hpp 和
src/gravity_compensation.cpp，生成 librebotarm_gravity.so。初始化读取完整 URDF、建立
Pinocchio model/data 和关节名称映射；compute() 接收 joint1..6 实际角度、返回补偿力矩。
夹爪手指在模型中固定为中立位置并保留质量；原始夹爪电机角度不等于手指位移。

硬件 write() 将 g(q) 加到额外 effort 的输出副本，不向命令缓冲反复累加。
目标位置、速度、MIT 增益和预计总力矩均有限制，但软件限幅不代替驱动器限流和机械保护。
当前电机角度与 URDF 关节角度按同方向、同零点、输出轴 rad 约定使用；力矩为输出轴 N*m。

启动普通 MoveIt demo：

```bash
source /home/zhao/rebot_ws/install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```

上层标准轨迹控制器使用 position、velocity 两个命令接口；kp/kd 使用 URDF 初值，
额外 effort 为零，硬件层持续计算并叠加重力。保持目标时 Kp>0 会产生拉回目标的力矩。
本版没有退出回零管理程序；直接关闭 launch 可能使能撤销，退出前应支撑机械臂。

