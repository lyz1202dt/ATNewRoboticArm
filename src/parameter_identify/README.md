# parameter_identify

`parameter_identify` 用于读取 `ParamterIdentify` 保存的测量 CSV，离线辨识机械臂动力学参数，并生成更新后的 URDF 和 YAML 报告。

本包只负责“根据 CSV 生成辨识结果”。激励轨迹生成和测量数据保存由 `arm_controller` 中的 `measure` 状态完成。

## 构建

```bash
cd /space2/Project/RoboticArm
colcon build --packages-select arm_controller parameter_identify --symlink-install
source install/setup.bash
```

运行时会从项目根目录加载完整 FIGAROH：

```text
third_party/figaroh-plus/src
```

## 录制 CSV

控制器参数中默认 CSV 路径是：

```yaml
measure_csv_file_path: /tmp/measured_for_identification.csv
measure_record_sample_rate: 500.0
```

也可以运行时修改：

```bash
ros2 param set /arm_controller measure_csv_file_path /tmp/measured_for_identification.csv
ros2 param set /arm_controller measure_record_sample_rate 500.0
ros2 param set /arm_controller exp_state measure
```

`measure` 状态完成激励轨迹后，会异步保存 CSV。CSV 表头格式为：

```text
time,pos_0,pos_1,...,vel_0,vel_1,...,torque_0,torque_1,...
```

## 生成辨识结果

```bash
ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf /space2/Project/RoboticArm/src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config /space2/Project/RoboticArm/src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

输出：

- `--output`：写入辨识后惯性参数的 URDF。
- `--report`：辨识过程、基参数、误差和物理一致性检查报告。

## 不安装时直接调试

```bash
cd /space2/Project/RoboticArm
PYTHONPATH=src/parameter_identify:$PYTHONPATH python3 -m parameter_identify.identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

## 常见参数

- `config/identify.yaml` 中的 `model.active_joints` 必须与 URDF 中参与辨识的关节名一致。
- `data.sample_time` 建议与 CSV 采样周期一致。例如 500 Hz 采样时为 `0.002`。
- `data.acceleration_source` 默认为 `computed`，会根据速度列计算加速度。
- `reconstruction.method: sdp` 需要可用的 SDP 求解依赖；如果只想先看诊断结果，可按需要调整 `reconstruction` 配置。
