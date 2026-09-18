# RoboticArm

基于 ROS 2 与 `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式，并内置运动学/动力学计算、状态机控制、以及动力学参数辨识（FIGAROH + CMA-ES）的完整工作流。

## 特性

- **双运行模式**：同一上层控制器 `ArmController` 可无缝切换
  - 仿真：`ArmController` → `SimPidController` → `mujoco_ros2_control/MujocoSystem`
  - 实机：`ArmController` → `ArmRealInterfaces` → USB CDC → MCU
- **基于状态机（FSM）的控制**：8 种可切换工作状态（见下方「状态机」）。
- **模型无关的计算库**：`ArmSolve → IKSolver → ModelBase + TaskMapping` 分层，任务空间语义与机器人模型数学解耦。
- **基于 Pinocchio 的刚体动力学**：正运动学、几何雅可比、RNEA 逆动力学、关节限位。
- **轨迹插值**：五次 Hermite/Bezier 段插值，保证位置、速度、加速度连续。
- **动力学参数辨识**：激励轨迹生成（傅里叶级数）、CSV 测量记录、离线辨识与 URDF 回写。
- **实时安全约束**：控制热路径严格无 ROS 参数读写、无动态内存分配。

## 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                        ArmController                         │
│        (controller_interface::ControllerInterface)           │
│  读取 URDF 关节名/default_kp/default_kd，每周期驱动 FSM        │
│  输出 position/velocity/effort/kp/kd/ki 命令                  │
│                                                              │
│     FSMArmControlFactory ── 状态机容器 + 共享运行态            │
│        ├── ModelFromURDF (Pinocchio, ModelBase 实现)          │
│        ├── Default6DofTaskSpaceMapping (TaskMapping 实现)     │
│        └── ArmSolve (IKSolver 封装)                           │
└──────────────────────────┬───────────────────────────────────┘
                           │ reference interfaces
              ┌────────────┴────────────┐
              │                         │
      ┌───────▼────────┐       ┌────────▼─────────┐
      │ SimPidController│       │ ArmRealInterfaces │
      │ (链式 PID)      │       │ (SystemInterface)  │
      └───────┬────────┘       │  CDCTrans (libusb) │
              │ effort          └────────┬─────────┘
      ┌───────▼────────┐                 │ packed struct
      │ mujoco_ros2_   │                 ▼
      │ control        │               MCU
      │ /MujocoSystem  │
      └────────────────┘
```

控制配置位于 `src/launch_pack/config/ros2_controller.yaml`，默认控制频率 **500 Hz**。

## 目录结构

```
RoboticArm/
├── src/
│   ├── arm_controller/         # 控制核心包（C++ 共享库 + 插件导出）
│   │   ├── controller/         #   ROS 2 controller 与 hardware interface 插件
│   │   ├── arm/                #   状态机（arm_fsm）与 URDF 模型封装（ModelFromURDF）
│   │   └── lib/
│   │       ├── calculate/      #   运动学/动力学/轨迹工具（ArmSolve/IKSolver/Trajectory）
│   │       ├── cdc_trans/      #   USB CDC 通信封装（libusb）
│   │       ├── fsm/            #   轻量 FSM 基类与工厂
│   │       ├── paramter_identify/ # 激励轨迹生成与测量（C++ 侧）
│   │       └── executer/       #   DAG 组件执行器实验代码（未接入主库）
│   ├── arm_model/              # URDF/STL/MJCF 模型描述包
│   ├── launch_pack/            # launch 文件、控制器 YAML、RViz 配置
│   ├── mujoco_ros2_control/    # MuJoCo 的 ros2_control 插件（上游 dfki-ric）
│   ├── parameter_identify/     # 离线动力学参数辨识（Python，ament_python）
│   └── robot_msgs/             # 自定义消息（MotorCmd/MotorState）
├── third_party/
│   ├── libcmaes/               # CMA-ES 优化库
│   └── figaroh-plus/           # FIGAROH 动力学参数辨识库
├── AGENT.md                    # 项目开发笔记（实时约束、约定等）
└── README.md
```

## 软件包

| 包 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller` | ament_cmake | 控制核心，通过 pluginlib 导出 3 个插件 |
| `arm_model` | ament_cmake | 机械臂 URDF 描述、STL 网格、MJCF 场景 |
| `launch_pack` | ament_cmake | launch 文件与控制器配置 |
| `mujoco_ros2_control` | ament_cmake | MuJoCo 仿真硬件插件（第三方） |
| `parameter_identify` | ament_python | 离线动力学参数辨识 CLI（`identify_arm`） |
| `robot_msgs` | ament_cmake | `MotorCmd.msg` / `MotorState.msg` |

### `arm_controller` 导出的插件

| 插件 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller/ArmController` | `controller_interface::ControllerInterface` | 上层控制器，读取 URDF 关节名与 `default_kp/default_kd`，每周期把状态送入 FSM，输出 `position/velocity/effort/kp/kd/ki` 命令 |
| `arm_controller/SimPidController` | `controller_interface::ChainableControllerInterface` | 链式 PID，接收每关节 6 个 reference interface，向仿真硬件输出 `effort` |
| `arm_controller/ArmRealInterfaces` | `hardware_interface::SystemInterface` | 实机硬件接口，固定 6 关节，使用 libusb CDC 与 MCU 收发 packed struct 数据包 |

## 状态机

状态机由 `FSMArmControlFactory` 注册，初始状态为 `idel`，通过 `exp_state` 参数切换：

| 状态名 | 功能 |
| --- | --- |
| `idel` | 机械臂锁定在当前位置 |
| `reset` | 机械臂复位 |
| `cart_traj` | 执行笛卡尔空间轨迹 |
| `joint_traj` | 执行关节空间轨迹 |
| `servo` | 伺服动作，接收速度指令并积分得到期望位置 |
| `admittance` | 导纳控制 |
| `teach_pendant` | 示教器，支持外力拖动、可配置阻尼与重力补偿 |
| `measure` | 系统参数辨识（激励轨迹执行 + 数据记录） |

运行时切换示例：

```bash
ros2 param set /arm_controller exp_state reset
```

## 依赖

- **ROS 2**（Humble / Ubuntu 22.04，Python 3.10）
- `ros2_control`：`controller_interface`、`hardware_interface`、`controller_manager`、`realtime_tools`
- `rclcpp`、`rclcpp_lifecycle`、`pluginlib`
- `Pinocchio`（运动学/动力学）
- `Eigen3`
- `libusb-1.0`（USB CDC 通信）
- `kdl_parser` / `orocos_kdl`
- `MuJoCo`（仿真）
- Python：`numpy`、`scipy`、`pinocchio`、`pyyaml`

## 构建

```bash
cd /space2/Project/RoboticArm
colcon build --symlink-install
source install/setup.bash
```

仅构建核心包与辨识包：

```bash
colcon build --packages-select arm_controller parameter_identify --symlink-install
```

## 运行

### 1. 仿真（MuJoCo）

```bash
source install/setup.bash
ros2 launch launch_pack arm_controller_test_sim.launch.py
```

### 2. 静态模型展示（无控制器）

```bash
ros2 launch launch_pack arm_static_display.launch.py
```

### 3. 实机

```bash
ros2 launch launch_pack arm_real.launch.py
```

> 实机模式由 `ArmRealInterfaces` 通过 USB CDC（VID `0x0483` / PID `0x5740`）与 MCU 通信，
> 数据包格式见 `controller/inc/data_pack.h`（`MCUTarget1Pack` / `MCUTarget2Pack` / `MCUStatePack`）。


## 参数辨识工作流

参数辨识由三部分协同完成，详细说明见 `src/parameter_identify/README.md`：

1. **激励轨迹生成 + 测量**：控制器 `measure` 状态（`ParamterMeasureState`）生成傅里叶激励轨迹并记录测量数据 CSV；
2. **离线辨识**：`parameter_identify` 包读取 CSV，基于 FIGAROH 辨识动力学参数；
3. **结果回写**：生成更新后的 URDF 与 YAML 报告。

快速流程：

```bash
# 进入测量状态，记录 CSV（默认 /tmp/measured_for_identification.csv）
ros2 param set /arm_controller exp_state measure

# 离线辨识
ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

## 开发约定

- 优先保持 `TaskMapping` 承担任务空间语义，`ModelBase` 承担机器人模型数学。
- 修改任务空间时，同步检查 `position_map()` 与 `jacobian_map()`。
- 修改控制器接口时，同步检查 controller YAML、URDF `ros2_control` 接口与插件 XML。
- 修改后优先构建验证：`colcon build --packages-select arm_controller`。

### ros2_control 实时约束

- `ArmController::update()` 是高频实时控制路径，禁止读写 ROS 参数、禁止动态内存操作。
- FSM 的 `enter()/run()/check_switch()/exit()` 均可能被 `update()` 间接调用，同属实时控制路径；
  状态机内需要的内存只能在构造函数或控制器非实时初始化阶段申请。
- FSM 的 `run()` 必须只读已缓存数据并写控制命令，不能调用 `get_parameter()`、不能构造临时
  `std::vector`、不能 `resize/reserve/assign/push_back`。
- FSM 的 `enter()` 只能重置标量状态、拷贝传感器/命令值到预分配 buffer、切换标志位。
- 运行时可更新的数组参数（如 `default_kp/default_kd/reset_joint_pos`）长度不得超过初始化时的
  `joints` 数量，避免状态切入时触发重新分配。

## 第三方库

以下第三方依赖以内嵌仓库方式放置在项目内：

- [`libcmaes`](https://github.com/CMA-ES/libcmaes)：CMA-ES 优化算法。
- [`figaroh-plus`](https://github.com/thanhndv212/figaroh-plus)：FIGAROH 动力学参数辨识。
- [`mujoco_ros2_control`](https://github.com/dfki-ric/mujoco_ros2_control)：MuJoCo 的 ros2_control 插件。
