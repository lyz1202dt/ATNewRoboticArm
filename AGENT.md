# RoboticArm 项目笔记

`src/arm_controller` 是机械臂控制核心包，构建共享库 `arm_controller`，
通过 pluginlib 导出控制器和硬件接口插件。

## 控制链路

- 仿真：`ArmController` -> `SimPidController` -> `mujoco_ros2_control/MujocoSystem`
- 实机：`ArmController` -> `ArmRealInterfaces` -> USB CDC -> MCU
- 控制配置在 `src/launch_pack/config/ros2_controller.yaml`，默认 500 Hz。

## 插件

- `arm_controller/ArmController`：上层控制器。读取 URDF 关节名和默认
  `default_kp/default_kd`，每周期把状态送入 FSM，再输出
  `position/velocity/effort/kp/kd/ki` 命令。
- `arm_controller/SimPidController`：链式 PID 控制器。接收每关节 6 个
  reference interface，向仿真硬件输出 `effort`。
- `arm_controller/ArmRealInterfaces`：实机 `hardware_interface::SystemInterface`。
  固定 6 关节，使用 libusb CDC 与 MCU 收发 packed struct 数据包。

## 主要目录

- `controller/`：ROS 2 controller 与 hardware interface 插件实现。
- `arm/`：机械臂控制状态机和 URDF 模型封装。
- `lib/calculate/`：运动学、动力学、轨迹工具。
- `lib/cdc_trans/`：USB CDC 通信封装。
- `lib/fsm/`：轻量 FSM 基类与工厂。
- `lib/executer/`：DAG 组件执行器实验代码，当前未接入主库。

## 计算库

`ArmSolve -> IKSolver -> ModelBase + TaskMapping`。
`ModelFromURDF` 基于 Pinocchio 提供 FK、几何雅可比、RNEA 和关节限位；
`IKSolver` 使用阻尼最小二乘迭代求 IK；
`Trajectory` 使用五次 Hermite/Bezier 段插值位置、速度和加速度。

## 开发约定

- 优先保持 `TaskMapping` 承担任务空间语义，`ModelBase` 承担机器人模型数学。
- 改任务空间时同步检查 `position_map()` 与 `jacobian_map()`。
- 改控制器接口时同步检查 controller YAML、URDF `ros2_control` 接口和插件 XML。
- 修改后优先构建验证：`colcon build --packages-select arm_controller`。
