# roboparty_deploy — 10-DOF 适配与调试记录

> 本文件用于记录当前 `/root/roboparty_deploy` 相对原生 Roboparty 的适配内容、已确认判断、当前问题定位和代码修改历史。  
> 结构约定：**项目与配置说明**、**已确认判断**、**下一步排查方向**、**代码修改记录** 分开记录，避免把结论和改动流水混在一起。

---

## 0. 当前结论摘要

当前部署目标是自研 **10-DOF 人形机器人**，基于 Roboparty 原生框架改造，硬件为：

- 电机：达妙 `DM_J8009_2EC × 6` + `DM_J10010_2EC × 4`
- CAN：`can3` 控制电机 1-5，`can0` 控制电机 6-10
- IMU：HIPNUC，`/dev/ttyUSB0`，115200 bps
- 策略：10-DOF ONNX，当前配置加载 `policy.onnx`

当前已确认：

| 项目 | 当前判断 |
|---|---|
| 当前策略 | 已从 `policy_stand.onnx` 切到 `policy.onnx`，`src` 与 `install` 配置一致 |
| 实机测试刚度 | 当前为第二档并降低踝关节：`kp=[30,30,60,60,30, 30,30,60,60,30]`，`kd=[0.3,0.3,0.6,0.6,0.3, 0.3,0.3,0.6,0.6,0.3]` |
| 动作缩放/裁剪 | 当前 `action_scale=0.08`、`clip_actions=3.0`、`act_alpha=0.10`，`src` 与 `install` 配置一致 |
| obs 顺序 | 与训练代码一致 |
| joint_default_angle | 与训练配置一致，临时改另一组默认角后更差，已回滚 |
| action_scale | 与训练配置一致，计算链路正确 |
| cmd_vel | 实机日志显示为 0，不是手柄漂移导致 |
| IMU gravity_b / ang_vel | 静态时基本正常 |
| 电机零点 | 用户确认：最开始已完成标定，A 键复位后位置已验证 |
| motor_sign | 用户确认：已在最开始标定完成，设置正确 |
| A 键复位 | 用户确认：使用 A 复位后的位置已验证 |

当前建议测试顺序：

> 当前已经进入策略落地调参阶段：优先通过日志曲线观察 `policy_raw / act_mapped / joint_pos / joint_error / ang_vel`，判断是动作幅值、跟踪误差、IMU摆动还是策略本身导致异常。

当前最值得继续查的方向：

> **策略闭环里使用的关节反馈是否真实、实时、且与训练坐标系一致。**

也就是说，若当前安全测试配置仍不能正常运行，下一步优先看 `joint_pos` / DM MIT 模式反馈刷新链路，而不是继续盲调 `kp/kd` 或继续怀疑 obs 顺序。

---

## 1. 项目与原版 Roboparty 的关键差异

本仓库是从原生 Roboparty fork 后，针对 10-DOF 自研人形机器人做的部署适配。

### 1.1 硬件差异

| 维度 | 原版 Roboparty | 当前项目 |
|---|---|---|
| 关节数 | 23 DOF | 10 DOF |
| 关节结构 | 双腿 + 上肢 | 双腿各 5 关节 |
| 电机 | 原版 DM 型号 | `DM_J8009_2EC` / `DM_J10010_2EC` |
| IMU | 原版默认 IMU | HIPNUC `/dev/ttyUSB0` 115200 |
| CAN 分配 | 原版默认 | `can3(1-5)` + `can0(6-10)` |
| 策略输入 | 原版多关节配置 | 39 维单帧 proprioception |

### 1.2 电机与关节映射

| 电机 ID | CAN | 型号 | 关节 |
|---:|---|---|---|
| 1 | can3 | DM_J8009_2EC | Left1_hip_yaw |
| 2 | can3 | DM_J8009_2EC | Left2_hip_roll |
| 3 | can3 | DM_J10010_2EC | Left3_hip_pitch |
| 4 | can3 | DM_J10010_2EC | Left4_knee |
| 5 | can3 | DM_J8009_2EC | Left5_ankle_pitch |
| 6 | can0 | DM_J8009_2EC | Right1_hip_yaw |
| 7 | can0 | DM_J8009_2EC | Right2_hip_roll |
| 8 | can0 | DM_J10010_2EC | Right3_hip_pitch |
| 9 | can0 | DM_J10010_2EC | Right4_knee |
| 10 | can0 | DM_J8009_2EC | Right5_ankle_pitch |

关键配置：

```yaml
motor_id: [1,2,3,4,5, 6,7,8,9,10]
motor_interface: ["can3", "can0"]
motor_num: [5, 5]
motor_model: [2,2,3,3,2, 2,2,3,3,2]
master_id_offset: 16
```

反馈帧 CAN ID：

```text
feedback_can_id = motor_id + master_id_offset
```

即 1-10 号电机反馈帧为 17-26。

### 1.3 运行配置

当前 `src/inference/config/inference.yaml` 与 install 下实际 launch 读取的配置一致：

```yaml
model_names: ["policy.onnx"]
obs_layouts:
  - "cmd_vel:3, ang_vel:3, gravity_b:3, dof_pos:10, dof_vel:10, last_action:10"
frame_stacks: [1]
act_alpha: 0.10
joint_num: 10
dt: 0.004
decimation: 5
action_scale: 0.08
clip_actions: 3.0
usd2urdf: [0,1,2,3,4,5,6,7,8,9]
joint_default_angle:
  [0.0, 0.0, -0.15, 0.33, -0.18,
   0.0, 0.0, 0.15, -0.33, 0.18]
```

`robot.yaml` 当前低风险实机测试阶段保持：

```yaml
kp: [30,30,60,60,30, 30,30,60,60,30]
kd: [0.3,0.3,0.6,0.6,0.3, 0.3,0.3,0.6,0.6,0.3]
motor_sign: [-1,1,-1,-1,-1, -1,1,-1,-1,-1]
urdf2motor: [1,2,3,4,5,6,7,8,9,10]
```

> 注意：当前 `kp/kd` 是低刚度安全测试值，不是最终运行刚度；若仍出现抽动或越限，不应直接升高刚度，应先查反馈实时性、反馈方向和策略目标范围。

---

## 2. 已确认判断与证据链

本节只记录“当前证据支持为正确”的判断。每条判断都标注到具体文件和行号范围，避免只写结论不写来源。它不等价于策略已经能稳定实机运行。

### 2.1 obs 顺序正确

**结论**：当前部署 `obs_layouts` 与训练代码 `compute_observations()` 的拼接顺序一致。

**训练代码出处**：本机训练文件 `策略/n2_10dof_env.py:159-168`：

```python
159: def compute_observations(self):
163:     self.commands[:, :3] * self.commands_scale,
164:     self.base_ang_vel  * self.obs_scales.ang_vel,
165:     self.projected_gravity,
166:     (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
167:     self.dof_vel * self.obs_scales.dof_vel,
168:     self.actions
```

对应语义：

```text
cmd_vel:3, ang_vel:3, gravity_b:3, dof_pos:10, dof_vel:10, last_action:10
```

**板端部署配置出处**：

- `/root/roboparty_deploy/src/inference/config/inference.yaml:4-5`
- `/root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml:4-5`

```yaml
4: obs_layouts:
5:   - "cmd_vel:3, ang_vel:3, gravity_b:3, dof_pos:10, dof_vel:10, last_action:10"
```

**板端执行代码出处**：`/root/roboparty_deploy/src/inference/src/obs_manager.cpp`：

- `obs_manager.cpp:50-51`：注册 `dof_pos`、`dof_vel` 观测段；
- `obs_manager.cpp:147-149`：`ang_vel` 从 `robot_->get_ang_vel()` 读取；
- `obs_manager.cpp:153-166`：`gravity_b` 由 IMU 四元数计算；
- `obs_manager.cpp:176-179`：`dof_pos = joint_pos - joint_default_angle`；
- `obs_manager.cpp:190-193`：`dof_vel = joint_vel * obs_scales_dof_vel`。

关键代码：

```cpp
147: ang_vel_buffer_ = robot_->get_ang_vel();
155: Eigen::Quaternionf q_b2w(quat_buffer_[0], quat_buffer_[1], quat_buffer_[2], quat_buffer_[3]);
157: Eigen::Quaternionf q_w2b = q_b2w.inverse();
179: segment[i] = (joint_pos_buffer_[usd2urdf_[i]] - joint_default_angle_[usd2urdf_[i]]) * obs_scales_dof_pos_;
193: segment[i] = joint_vel_buffer_[usd2urdf_[i]] * obs_scales_dof_vel_;
```

因此判断：**obs 字段顺序与训练一致，当前没有证据表明 obs 顺序错位。**

### 2.2 joint_default_angle 正确

**结论**：当前部署的 `joint_default_angle` 与训练配置中的 `default_joint_angles` 一致。

**训练代码出处**：本机训练文件 `策略/n2_10dof_config.py:26-38`：

```python
26: default_joint_angles = {
27:     "Left1_hip_yaw_Joint": 0.0,
28:     "Left2_hip_roll_Joint": 0.0,
29:     "Left3_hip_pitch_Joint": -0.15,
30:     "Left4_knee_Joint": 0.33,
31:     "Left5_ankle_pitch_Joint": -0.18,
32:     "Right1_hip_yaw_Joint": 0.0,
33:     "Right2_hip_roll_Joint": 0.0,
34:     "Right3_hip_pitch_Joint": 0.15,
35:     "Right4_knee_Joint": -0.33,
36:     "Right5_ankle_pitch_Joint": 0.18,
37: }
```

**板端部署配置出处**：

- `/root/roboparty_deploy/src/inference/config/inference.yaml:26-28`
- `/root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml:26-28`

```yaml
26: joint_default_angle:
27:   [0.0, 0.0, -0.15, 0.33, -0.18,
28:    0.0, 0.0, 0.15, -0.33, 0.18]
```

**反证记录**：曾临时改为：

```text
[0.48, -0.17, 0.26, -0.26, -0.04,
 0.48, -0.34, 0.39, -0.39, -0.04]
```

运行后 `policy_raw` 和 `act_mapped` 明显变大，多个目标角超出 `joint_limits`，因此该临时配置已回滚。

当前判断：**原始 `joint_default_angle` 与当前策略训练配置一致。**

### 2.3 action_scale 正确

**结论**：当前部署 `action_scale = 0.22` 与训练配置一致，且从 `policy_raw` 到 `act_mapped` 的计算链路正确。

**训练代码出处**：本机训练文件 `策略/n2_10dof_config.py:145-146`：

```python
145: # 动作缩放因子: 目标角度 = actionScale * action + defaultAngle
146: action_scale = 0.22
```

**板端部署配置出处**：

- `/root/roboparty_deploy/src/inference/config/inference.yaml:19`
- `/root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml:19`

```yaml
19: action_scale: 0.22
```

**板端执行代码出处**：`/root/roboparty_deploy/src/inference/src/inference_node.cpp:346-348`：

```cpp
346: policy.ctx->output_buffer[i] = std::clamp(policy.ctx->output_buffer[i], -clip_actions_, clip_actions_);
347: act_[usd2urdf_[i]] = policy.ctx->output_buffer[i];
348: act_[usd2urdf_[i]] = act_[usd2urdf_[i]] * action_scale_ + joint_default_angle_[usd2urdf_[i]];
```

**终端日志数值验证**：曾观察到：

```text
policy_raw[3] ≈ 2.839
act_mapped[3] ≈ 0.955
```

按代码公式计算：

```text
2.839 * 0.22 + 0.33 = 0.95458
```

与终端 `act_mapped[3]` 基本一致。因此判断：**策略输出到目标角度的缩放和偏置计算正确。**

### 2.4 cmd_vel 当前为 0

**结论**：当前实机运行时，策略输入中的速度命令为 0；机器人异常动作不是由手柄速度命令非零直接导致。

**训练代码出处**：本机训练文件 `策略/n2_10dof_env.py:136-143`：

```python
136: def _resample_commands(self, env_ids):
137:     """原地踏步任务：保持命令维度不变，但目标速度和偏航角速度固定为0。"""
142:     self.commands[env_ids, :] = 0.0
```

**训练配置出处**：本机训练文件 `策略/n2_10dof_config.py:315-319`：

```python
315: lin_vel_x = [0.0, 0.0]
317: lin_vel_y = [0.0, 0.0]
319: ang_vel_yaw = [0.0, 0.0]
```

**板端调试打印代码出处**：`/root/roboparty_deploy/src/inference/src/inference_node.cpp:336`：

```cpp
336: RCLCPP_INFO(this->get_logger(), "cmd_vel[x,y,yaw]: %s", oss_cmd_vel.str().c_str());
```

**终端日志依据**：多次运行日志中出现：

```text
cmd_vel[x,y,yaw]: 0.000 -0.000 0.000
```

因此判断：**当前没有手柄漂移或速度命令非零导致策略进入行走/下蹲输出的证据。**

### 2.5 IMU 当前基本正常

**结论**：当前日志显示 IMU 重力方向和角速度基本正常；暂不认为 IMU 是主要异常来源。

**板端代码出处**：`/root/roboparty_deploy/src/inference/src/obs_manager.cpp:146-166`：

```cpp
146: void InferenceNode::get_ang_vel_obs(std::vector<float>& segment) {
147:     ang_vel_buffer_ = robot_->get_ang_vel();
148:     for (int i = 0; i < 3; i++) {
149:         segment[i] = ang_vel_buffer_[i] * obs_scales_ang_vel_;
153: void InferenceNode::get_gravity_b_obs(std::vector<float>& segment) {
154:     quat_buffer_ = robot_->get_quat();
155:     Eigen::Quaternionf q_b2w(quat_buffer_[0], quat_buffer_[1], quat_buffer_[2], quat_buffer_[3]);
156:     Eigen::Vector3f gravity_w(0.0f, 0.0f, -1.0f);
157:     Eigen::Quaternionf q_w2b = q_b2w.inverse();
158:     Eigen::Vector3f gravity_b = q_w2b * gravity_w;
```

**板端调试打印代码出处**：`/root/roboparty_deploy/src/inference/src/inference_node.cpp:338-340`：

```cpp
338: RCLCPP_INFO(this->get_logger(), "imu_quat[w,x,y,z]: %s", oss_quat.str().c_str());
339: RCLCPP_INFO(this->get_logger(), "gravity_b[x,y,z]: %s", oss_gravity.str().c_str());
340: RCLCPP_INFO(this->get_logger(), "ang_vel[x,y,z]: %s", oss_ang_vel.str().c_str());
```

**终端日志依据**：静止时多次观察到：

```text
gravity_b[x,y,z]: -0.136 -0.074 -0.988
ang_vel[x,y,z]: 0.000 -0.000 0.000
```

判断标准：机器人近似直立时，`gravity_b` 应接近 `[0, 0, -1]`，角速度应接近 `[0, 0, 0]`。当前值存在小幅姿态倾斜，但整体接近预期。因此判断：**IMU 姿态和角速度当前基本可信。**

### 2.6 零点、motor_sign 与 A 复位

用户确认：

1. 电机零点在最开始已经标定完成；
2. `motor_sign` 在最开始已经确认并设置正确；
3. 使用 A 键复位后的实际位置已经验证；
4. 因此当前排查不再优先怀疑零点和 `motor_sign` 的配置值本身。

仍需注意：这不等于 `joint_pos` 在策略运行时一定实时刷新，也不等于反馈方向在 obs 链路中一定已经等价于训练坐标系。后续要查的是“反馈链路实时性与闭环一致性”，不是重新否定初始标定。

---

## 3. 当前问题现象

在 `kp/kd = 0` 的安全状态下运行 `policy_stand.onnx`：

```text
cmd_vel[x,y,yaw]: 0.000 -0.000 0.000
gravity_b[x,y,z]: -0.136 -0.074 -0.988
ang_vel[x,y,z]: 0.000 -0.000 0.000
joint_pos[0..9]: 0.101 -0.094 0.234 0.279 -0.429 0.070 -0.020 -0.103 -0.668 0.167
```

但策略仍持续输出较大动作，例如：

```text
policy_raw[0..9]: -0.071 -0.326 -0.405 2.837 -1.949 0.008 -0.679 2.627 -1.348 1.652
act_mapped[0..9]: -0.016 -0.072 -0.239 0.954 -0.609 0.002 -0.149 0.728 -0.627 0.543
```

说明：

- obs 顺序、默认角、action scale、cmd_vel、IMU 当前看起来都不是主因；
- 策略输出目标本身较大；
- 由于 `kp/kd = 0`，此时电机不会真实跟随目标，主要用于观测策略输出；
- 一旦恢复高 `kp/kd`，这些目标可能导致机器人快速动作甚至抽动。

---

## 4. 下一步优先排查方向

### 4.1 优先级 1：确认 joint_pos 是否实时刷新

当前日志里 `joint_pos[0..9]` 长时间固定：

```text
0.101 -0.094 0.234 0.279 -0.429 0.070 -0.020 -0.103 -0.668 0.167
```

下一步应确认：

1. DM 电机 MIT 模式下 `joint_pos` 是否会随真实物理动作刷新；
2. `refresh_motor_status()` 是否需要周期性调用；
3. `listen_motors.py` 纯监听模式是否只能在总线上有控制/查询帧时更新；
4. 策略运行时 `get_joint_q()` 读到的是不是旧反馈。

建议检查或实验：

- 在不跑 ONNX 策略时，周期性主动查询 DM 状态；
- 手动轻微转动电机，观察反馈是否实时变化；
- 对比 `listen_motors.py`、`joint_pos` 打印、驱动内部 `motor_pos_` 三者是否一致。

### 4.2 优先级 2：验证反馈方向是否与训练坐标系一致

即使 `motor_sign` 下发方向已确认，也仍需确认 obs 反馈方向：

```cpp
segment[i] = joint_pos_buffer_[usd2urdf_[i]] - joint_default_angle_[usd2urdf_[i]];
```

当前 `motor_sign` 明确参与下发：

```cpp
motor_pos_target_[idx] * robot_cfg_->motor_sign_[idx]
```

但 obs 里的 `dof_pos` 没有显式乘 `motor_sign`。如果驱动返回的 `joint_pos` 已经是策略坐标系，这没问题；如果返回的是电机物理编码器方向，则可能闭环方向不一致。

下一步建议做小幅低刚度单关节实验：

1. 不跑 ONNX；
2. 低 `kp/kd`；
3. 对单个关节下发 `default + 0.05 rad`；
4. 观察真实动作方向与训练正方向是否一致；
5. 同时观察 `joint_pos` 的变化方向是否与策略坐标一致。

### 4.3 优先级 3：确认策略模型是否为预期版本

本机策略目录和板端模型已有多份：

```text
policy.onnx
policy_stand.onnx
policy_test.onnx
policy_amp.onnx
policy_dance.onnx
policy_getup.onnx
...
```

已确认 `policy.onnx`、`policy_stand.onnx`、`policy_test.onnx` MD5 不同，因此不是同一个文件。但仍需确认：

1. `policy_stand.onnx` 是否确实是最新导出的站立策略；
2. 导出时使用的 config 是否就是当前 `n2_10dof_config.py` / `n2_10dof_env.py`；
3. 是否存在训练时使用 frame_stack、归一化、观测裁剪或额外处理，而部署端未同步。

### 4.4 调参能否解决？

当前阶段不建议先靠调参硬压。

如果问题是：

- 反馈不实时；
- 反馈方向和训练坐标系不一致；
- 策略模型版本不对；
- 真实电机闭环和训练假设不一致；

那么只调 `kp/kd`、`act_alpha`、`action_scale` 只能缓解，不能根治。

调参适合作为“方向和反馈都确认正确之后”的步骤，例如：

```yaml
action_scale: 0.22 -> 0.08~0.12
act_alpha: 0.05
kp/kd: 从低刚度逐级增加
clip_actions: 100 -> 3 或 5
```

---

## 5. 代码修改记录

### 记录约定

后续所有代码修改、配置修改、运行实验和结论更新都分层记录：

1. **代码/配置修改记录**：记录修改原因、修改文件、具体参数或代码点、是否需要重新编译、验证方式；
2. **运行实验记录**：记录测试时间、启动命令、当前模型、`kp/kd`、`clip_actions`、机器人支撑状态、关键日志和实际现象；
3. **问题定位结论**：只记录由实验支持的判断，避免把猜测写成结论；
4. **下一步动作**：记录下一轮只做什么、先不做什么，以及安全注意事项。

每次运行后优先补充到“运行实验记录”；每次改代码或改 YAML 后补充到“代码修改记录”。若运行结果改变当前判断，再同步更新“当前结论摘要”和“下一步排查方向”。

### 5.1 修复 10 关节策略 usd2urdf 越界问题

时间：2026-06-27 15:02

#### 原因

`joint_num = 10` 时，`usd2urdf` 原为 1-based：

```yaml
usd2urdf: [1,2,3,4,5,6,7,8,9,10]
```

但 C++ 使用 0-based 下标访问，会导致：

1. 第 0 个关节没有被策略输出更新；
2. 最后一项访问下标 10，越界；
3. `dof_pos`、`dof_vel`、动作映射都有潜在错误。

#### 修改

将 `usd2urdf` 改为：

```yaml
usd2urdf: [0,1,2,3,4,5,6,7,8,9]
```

并在 `src/inference/src/ros_interface.cpp` 增加配置校验：

- `joint_num > 0`
- `usd2urdf.size() == joint_num`
- `usd2urdf` 每项在 `[0, joint_num - 1]`
- `usd2urdf` 不允许重复
- `joint_default_angle.size() == joint_num`
- `joint_limits.size() == joint_num * 2`
- `clip_cmd.size() == 6`

#### 验证

已检查所有 `inference*.yaml`，`usd2urdf` 无越界、无重复。

---

### 5.2 增加策略动作映射调试打印

时间：2026-06-27 15:40

#### 修改文件

```text
src/inference/src/inference_node.cpp
```

#### 新增打印

```text
policy_raw[0..9]
act_mapped[0..9]
```

用途：确认 ONNX 原始输出与下发目标角之间的映射关系。

计算公式：

```text
act_mapped[i] = policy_raw[i] * action_scale + joint_default_angle[i]
```

---

### 5.3 增加关节反馈与 IMU 调试打印

时间：2026-06-27 15:55

#### 修改文件

```text
src/inference/src/inference_node.cpp
```

#### 新增打印

```text
joint_pos[0..9]
imu_quat[w,x,y,z]
gravity_b[x,y,z]
ang_vel[x,y,z]
```

用途：排查策略输入中的关节反馈、IMU 姿态、机体系重力方向和角速度是否异常。

---

### 5.4 临时修改 joint_default_angle 并回滚

时间：2026-06-27 16:20

#### 修改原因

怀疑 `joint_default_angle` 与训练默认姿态不一致，临时将默认角改为另一组关节表默认值。

#### 临时值

```yaml
joint_default_angle:
  [0.48, -0.17, 0.26, -0.26, -0.04,
   0.48, -0.34, 0.39, -0.39, -0.04]
```

#### 验证结果

运行后 `policy_raw` 与 `act_mapped` 变得更大，多个 `act_mapped` 超出 `joint_limits`。

#### 当前状态

已回滚为：

```yaml
joint_default_angle:
  [0.0, 0.0, -0.15, 0.33, -0.18,
   0.0, 0.0, 0.15, -0.33, 0.18]
```

---

### 5.5 增加 cmd_vel 调试打印

时间：2026-06-27 20:50

#### 修改文件

```text
src/inference/src/inference_node.cpp
```

#### 新增打印

```text
cmd_vel[x,y,yaw]
```

当前同周期调试输出：

```text
policy_raw[0..9]
cmd_vel[x,y,yaw]
joint_pos[0..9]
imu_quat[w,x,y,z]
gravity_b[x,y,z]
ang_vel[x,y,z]
act_mapped[0..9]
```

#### 验证

二进制已包含：

```text
cmd_vel[x,y,yaw]: %s
```

---

## 6. 编译注意事项

RDK X5 上并行编译 `roboparty_inference` 容易出现 `cc1plus` 被 OOM killer 杀掉。建议使用单线程：

```bash
source /opt/tros/humble/setup.bash
cd /root/roboparty_deploy
MAKEFLAGS="-j1" colcon build --packages-select roboparty_inference --symlink-install --executor sequential
```

如果通过 RDK Studio 后台 SSH 编译遇到 SIGHUP，可用 screen：

```bash
screen -dmS build_inference bash -lc 'source /opt/tros/humble/setup.bash && cd /root/roboparty_deploy && MAKEFLAGS="-j1" colcon build --packages-select roboparty_inference --symlink-install --executor sequential 2>&1 | tee /tmp/roboparty_inference_build.log; exec bash'
```

---

## 7. 常用查看命令

### 查看 inference screen 输出

```bash
screen -r inference_session
```

### 导出 screen 历史

```bash
screen -S inference_session -X hardcopy -h /tmp/inference_log.txt
strings -a /tmp/inference_log.txt | grep -E "policy_raw|cmd_vel|joint_pos|imu_quat|gravity_b|ang_vel|act_mapped" | tail -120
```

### 查看当前模型配置

```bash
grep -nE "model_names|obs_layouts|act_alpha|action_scale|usd2urdf|joint_default_angle" \
  /root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml
```

### 查看模型文件 MD5

```bash
cd /root/roboparty_deploy/src/inference/models
md5sum policy.onnx policy_stand.onnx policy_test.onnx
```

---

### 5.6 备份当前工程到 KINGSTON U 盘，并增强 listen_motors.py

时间：2026-06-29 11:26

#### 备份

在修改 `listen_motors.py` 前，已按要求将当前 `/root/roboparty_deploy` 完整备份到板端 U 盘 `KINGSTON`：

```text
/media/KINGSTON/roboparty_deploy_backup_20260629_112607.tar.gz
```

备份大小：

```text
186M
```

备份校验：

```bash
tar -tzf /media/KINGSTON/roboparty_deploy_backup_20260629_112607.tar.gz >/tmp/roboparty_backup_filelist.txt
```

校验结果：压缩包可读，文件列表 3670 项，确认包含关键文件：

```text
roboparty_deploy/read_me.md
roboparty_deploy/scripts/listen_motors.py
roboparty_deploy/src/inference/src/inference_node.cpp
```

#### 修改原因

当前 `listen_motors.py` 默认是纯监听脚本，只在 CAN 总线上已有反馈帧时刷新显示。按 X 使能后只有初始化/刷新瞬间能看到位置变化，之后若没有控制/查询帧，显示不会持续变化。

为排查 DM MIT 模式下 `joint_pos` 是否能随物理转动实时刷新，给脚本增加主动查询模式。

#### 修改文件

```text
scripts/listen_motors.py
```

#### 新增功能

保留默认纯监听模式：

```bash
python3 scripts/listen_motors.py
```

新增主动查询模式：

```bash
python3 scripts/listen_motors.py --active
```

主动模式会周期性向电机发送与 `DmMotorDriver::refresh_motor_status()` 一致的查询帧：

```text
can_id = 0x7FF
data = [motor_id_low, motor_id_high, 0xCC, 0, 0, 0, 0, 0]
```

可指定查询频率：

```bash
python3 scripts/listen_motors.py --active --rate 20
```

可关闭清屏刷新，便于记录日志：

```bash
python3 scripts/listen_motors.py --active --no-clear
```

#### 使用注意

`--active` 会向 CAN 总线发送查询帧，**不要和 ROS 控制/推理节点同时使用**，避免干扰控制。推荐流程：

```bash
# 1. 停掉推理/控制节点
screen -S inference_session -X quit

# 2. 单独运行主动查询监听
cd /root/roboparty_deploy
python3 scripts/listen_motors.py --active

# 3. 手动轻轻转动某个关节，观察对应 ID 的位置是否连续变化
```

#### 验证

已完成语法检查：

```bash
python3 -m py_compile /root/roboparty_deploy/scripts/listen_motors.py
```

已确认帮助信息可正常输出：

```bash
python3 /root/roboparty_deploy/scripts/listen_motors.py --help
```

---

### 5.7 新增单关节小幅方向测试脚本

时间：2026-06-29

#### 修改文件

```text
scripts/test_joint_direction.py
```

#### 目的

在不运行 ONNX 策略的情况下，只对单个关节下发 `default ± delta` 的小幅 MIT 位置命令，用于确认：

1. 动的是不是目标关节；
2. `+delta` / `-delta` 的物理方向是否符合训练坐标系；
3. 电机反馈位置是否跟随目标变化；
4. 策略坐标系、`motor_sign` 下发方向和反馈方向是否闭环一致。

#### 安全设计

- 默认 dry-run，只打印测试计划，不发送控制命令；
- 必须显式加 `--execute` 才会实际使能电机并发送 MIT 命令；
- 每次只测试一个关节；
- 默认使用低刚度：
  - `DM_J8009_2EC`：`kp=10, kd=0.3`
  - `DM_J10010_2EC`：`kp=20, kd=0.6`
- 每一步都需要用户按 Enter 确认；
- 动作异常、方向不对、有异响时立即 `Ctrl+C`。

#### 使用前准备

必须先 source ROS/TROS 和项目环境，否则 Python 找不到 `motors_py`：

```bash
cd /root/roboparty_deploy
source /opt/tros/humble/setup.bash
source install/setup.bash
```

确保没有运行推理/控制节点：

```bash
screen -S inference_session -X quit 2>/dev/null || true
```

不要同时运行：

```bash
python3 scripts/listen_motors.py --active
```

#### dry-run 预览

以 4 号关节 `Left4_knee` 为例：

```bash
python3 scripts/test_joint_direction.py --joint 4
```

输出会显示：

```text
策略坐标目标: default, default+delta, default, default-delta, default
电机实际下发: 上述目标 * motor_sign
```

#### 实际执行

确认机器人已安全支撑/悬空/有人扶持后执行：

```bash
python3 scripts/test_joint_direction.py --joint 4 --execute
```

可调整测试幅度：

```bash
python3 scripts/test_joint_direction.py --joint 4 --delta 0.03 --execute
```

可手动降低刚度：

```bash
python3 scripts/test_joint_direction.py --joint 4 --delta 0.03 --kp 10 --kd 0.3 --execute
```

#### 推荐测试顺序

优先测试大动作相关关节：

```bash
python3 scripts/test_joint_direction.py --joint 4 --execute   # Left4_knee
python3 scripts/test_joint_direction.py --joint 8 --execute   # Right3_hip_pitch
python3 scripts/test_joint_direction.py --joint 5 --execute   # Left5_ankle_pitch
python3 scripts/test_joint_direction.py --joint 10 --execute  # Right5_ankle_pitch
```

再补全其余关节：

```bash
for i in 1 2 3 4 5 6 7 8 9 10; do
  python3 scripts/test_joint_direction.py --joint $i
  # 确认 dry-run 计划无误后，再单独加 --execute 执行
done
```

#### 判断记录建议

测试每个关节时记录：

```text
joint_id:
关节名:
+delta 实际方向:
-delta 实际方向:
反馈 pos 是否变化:
反馈变化方向是否与策略坐标一致:
是否异常:
```

如果某个关节 `+delta` 真实方向与训练正方向相反，或反馈变化方向与策略坐标相反，需要继续检查该关节的 `motor_sign`、反馈转换链路和驱动坐标系。

---

### 5.8 切换到新策略 policy.onnx 并恢复运行 kp/kd

时间：2026-06-29

#### 修改原因

用户确认当前 `policy.onnx` 是新加入的策略，需要从 `policy_stand.onnx` 切换到 `policy.onnx` 进行实机测试；同时将调试阶段置零的 `kp/kd` 恢复为运行参数。

#### 修改文件

```text
src/inference/config/inference.yaml
install/roboparty_inference/share/roboparty_inference/config/inference.yaml
src/inference/config/robot.yaml
```

#### 修改内容

策略模型切换为：

```yaml
model_names: ["policy.onnx"]
```

`kp/kd` 从全 0 恢复为左右腿一致的运行参数：

```yaml
kp:
  [150.0, 150.0, 300.0, 300.0, 300.0,
   150.0, 150.0, 300.0, 300.0, 300.0]
kd:
  [1.5, 1.5, 3.0, 3.0, 3.0,
   1.5, 1.5, 3.0, 3.0, 3.0]
```

#### 注意

本次只修改 YAML 配置，不需要重新编译。需要重启 `start_robot.sh` / inference launch 后生效。

---

### 5.9 为循环下蹲策略降低实机测试刚度与动作裁剪

时间：2026-06-29

#### 修改原因

当前 `policy.onnx` 是新加入的循环下蹲策略。用户反馈恢复高刚度后，一运行策略机器人就抽动，并出现 `Joint 2 out of limit`。为降低实机风险，先将运行参数降为安全测试值，观察策略目标与真实反馈趋势。

#### 修改文件

```text
src/inference/config/robot.yaml
src/inference/config/inference.yaml
install/roboparty_inference/share/roboparty_inference/config/inference.yaml
```

#### 修改内容

`kp/kd` 从运行刚度：

```yaml
kp:
  [150.0, 150.0, 300.0, 300.0, 300.0,
   150.0, 150.0, 300.0, 300.0, 300.0]
kd:
  [1.5, 1.5, 3.0, 3.0, 3.0,
   1.5, 1.5, 3.0, 3.0, 3.0]
```

降为安全测试值：

```yaml
kp:
  [10.0, 10.0, 20.0, 20.0, 20.0,
   10.0, 10.0, 20.0, 20.0, 20.0]
kd:
  [0.3, 0.3, 0.6, 0.6, 0.6,
   0.3, 0.3, 0.6, 0.6, 0.6]
```

`clip_actions` 从：

```yaml
clip_actions: 100.0
```

降为：

```yaml
clip_actions: 3.0
```

`act_alpha` 保持：

```yaml
act_alpha: 0.05
```

#### 注意

本次只修改 YAML 配置，不需要重新编译。需要重启 `start_robot.sh` / inference launch 后生效。若低刚度下仍出现 out-of-limit，应继续排查初始姿态、策略目标幅度、反馈实时性和限位方向，而不是直接恢复高刚度。

---

### 5.9 循环下蹲策略安全降参测试

时间：2026-06-29

#### 修改原因

当前 `policy.onnx` 是新加入的循环下蹲策略。使用运行刚度：

```yaml
kp: [150,150,300,300,300, 150,150,300,300,300]
kd: [1.5,1.5,3,3,3, 1.5,1.5,3,3,3]
```

启动策略后机器人立即抽动，并出现 `Joint 2 out of limit`。为了降低实机风险，先切到安全测试参数，观察策略输出和关节反馈趋势。

#### 当前策略

```yaml
model_names: ["policy.onnx"]
```

#### 安全测试参数

`src/inference/config/robot.yaml` 当前：

```yaml
kp:
  [10.0, 10.0, 20.0, 20.0, 20.0,
   10.0, 10.0, 20.0, 20.0, 20.0]
kd:
  [0.3, 0.3, 0.6, 0.6, 0.6,
   0.3, 0.3, 0.6, 0.6, 0.6]
```

`src/inference/config/inference.yaml` 与 install 配置当前：

```yaml
clip_actions: 3.0
act_alpha: 0.05
```

#### 验证命令

```bash
grep -n "model_names" /root/roboparty_deploy/src/inference/config/inference.yaml \
  /root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml

grep -n "clip_actions" /root/roboparty_deploy/src/inference/config/inference.yaml \
  /root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml

sed -n '25,32p' /root/roboparty_deploy/src/inference/config/robot.yaml
```

#### 注意

本次只修改 YAML 配置，不需要重新编译。重启 `start_robot.sh` / inference launch 后生效。测试时仍需有人扶持/保护机器人，若出现异常动作或 out-of-limit，立即停止。

---

### 5.10 继续提高循环下蹲策略 kp/kd 到第二档

时间：2026-06-29

#### 修改原因

上一档安全参数：

```yaml
kp: [10,10,20,20,20, 10,10,20,20,20]
kd: [0.3,0.3,0.6,0.6,0.6, 0.3,0.3,0.6,0.6,0.6]
```

实机测试结果：机器人已经可以执行下蹲趋势，但支撑刚度不足，表现为站不住。判断当前优先提高 `kp/kd`，暂时保持 `act_alpha = 0.05` 不变，避免目标变化速度突然增大。

#### 修改文件

```text
src/inference/config/robot.yaml
```

#### 当前参数

```yaml
kp:
  [30.0, 30.0, 60.0, 60.0, 60.0,
   30.0, 30.0, 60.0, 60.0, 60.0]
kd:
  [0.8, 0.8, 1.5, 1.5, 1.5,
   0.8, 0.8, 1.5, 1.5, 1.5]
```

#### 保持不变

```yaml
model_names: ["policy.onnx"]
clip_actions: 3.0
act_alpha: 0.05
action_scale: 0.22
```

#### 关于 act_alpha

`act_alpha` 不是策略输出本身的比例系数，而是目标角的时间滤波系数。代码位置：

```cpp
/root/roboparty_deploy/src/inference/src/inference_node.cpp:229
last_act_[i] = act_alpha_ * act_[i] + (1 - act_alpha_) * last_act_[i];
```

含义：

- `act_[i]` 是当前策略输出换算后的目标角；
- `last_act_[i]` 是实际传给控制线程的平滑目标；
- `act_alpha` 越大，`last_act` 越快追上策略目标；
- `act_alpha` 越小，动作越慢、更平滑，但会引入相位滞后。

因此，不加大 `act_alpha` 并不代表策略输出的“1”不对应。策略输出仍然通过：

```cpp
act = policy_raw * action_scale + joint_default_angle
```

正确换算为目标角；只是实际下发目标 `last_act` 会慢慢追过去。当前问题是支撑刚度不足，所以先提高 `kp/kd`，暂时不提高 `act_alpha`。

---

### 5.11 训练频率与部署频率核对

时间：2026-06-29

#### 核对原因

实机运行循环下蹲策略时出现下蹲后内八、后续 `Joint out of limit`。怀疑训练频率与部署频率不一致可能导致策略节奏、相位或控制跟踪异常，因此核对训练侧和部署侧的 `dt / decimation`。

#### 训练侧配置

本机训练目录 `策略` 中配置显示：

```python
# 策略/2_10dof_config.py
control.decimation = 10
sim.dt = 0.002
```

因此训练侧：

```text
仿真/底层控制步长 = 0.002 s = 500 Hz
策略更新周期 = dt * decimation = 0.002 * 10 = 0.02 s = 50 Hz
```

#### 部署侧配置

板端部署配置：

```yaml
# /root/roboparty_deploy/src/inference/config/inference.yaml
# /root/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/config/inference.yaml
dt: 0.004
decimation: 5
```

运行启动日志也确认：

```text
decimation: 5
dt: 0.004000
```

推理代码中：

```cpp
// /root/roboparty_deploy/src/inference/src/inference_node.cpp
control period = dt
inference period = dt * decimation
```

因此部署侧：

```text
底层控制周期 = 0.004 s = 250 Hz
策略更新周期 = dt * decimation = 0.004 * 5 = 0.02 s = 50 Hz
```

#### 判断

| 项目 | 训练侧 | 部署侧 | 是否一致 |
|---|---:|---:|---|
| 策略更新频率 | 50 Hz | 50 Hz | 一致 |
| 策略更新周期 | 20 ms | 20 ms | 一致 |
| 底层控制频率 | 500 Hz | 250 Hz | 不一致 |
| 底层控制周期 | 2 ms | 4 ms | 不一致 |

当前判断：

1. **策略频率是对上的**，不是“策略每秒执行次数不一致”导致的主要问题；
2. **底层控制频率比训练仿真低一半**，可能影响 PD 跟踪、接触相位和动态稳定性，但它不是最明显的主因；
3. 如果 sim2sim 中也出现类似内八，优先怀疑策略/奖励/动作幅度，而不是部署频率；
4. 不建议立即把部署 `dt` 改成 0.002，因为这会把控制线程提高到 500 Hz，增加 CPU、CAN 和实时调度压力；若要测试，必须在低 `kp/kd`、低 `action_scale`、吊起/扶持状态下单独验证。

#### 后续建议

当前优先保持部署频率不变：

```yaml
dt: 0.004
decimation: 5
```

先按真实样机调试流程继续调：

```yaml
action_scale: 0.10
clip_actions: 1.5
act_alpha: 0.10
```

如果后续仍怀疑频率影响，再做一个单独实验分支：

```yaml
dt: 0.002
decimation: 10
```

并对比：

- 是否出现 `Inference loop overran`；
- CAN 是否稳定；
- 关节是否更跟脚；
- 下蹲相位是否改善；
- 是否更容易抖动或过热。

---

### 5.12 按真实样机调试说明降低动作幅值并小幅提高 act_alpha

时间：2026-06-29

#### 修改原因

当前 `policy.onnx` 循环下蹲策略在第二档 `kp/kd` 下已经可以下蹲，但后续出现内八趋势和 `Joint out of limit` 风险。参考真实样机部署调试说明，初次实机策略测试应降低动作幅值和动作裁剪，避免策略输出过大直接冲击限位；同时将 `act_alpha` 从过低的 0.05 小幅提高到 0.10，减轻周期动作的相位滞后，但不直接提高到 0.2 以上。

#### 修改文件

```text
src/inference/config/inference.yaml
install/roboparty_inference/share/roboparty_inference/config/inference.yaml
```

#### 修改内容

```yaml
act_alpha: 0.10
action_scale: 0.10
clip_actions: 1.5
```

#### 保持不变

```yaml
model_names: ["policy.onnx"]
dt: 0.004
decimation: 5
```

`src/inference/config/robot.yaml` 保持第二档刚度：

```yaml
kp:
  [30.0, 30.0, 60.0, 60.0, 60.0,
   30.0, 30.0, 60.0, 60.0, 60.0]
kd:
  [0.8, 0.8, 1.5, 1.5, 1.5,
   0.8, 0.8, 1.5, 1.5, 1.5]
```

#### 预期效果

- `action_scale` 从 0.22 降到 0.10：将策略动作幅值降低到约 45%；
- `clip_actions` 从 3.0 降到 1.5：限制 policy_raw 大值导致的目标角冲击；
- `act_alpha` 从 0.05 提到 0.10：减少动作滞后，但仍保留一定平滑。

#### 验证重点

重启后观察：

```text
policy_raw
cmd_vel
joint_pos
act_mapped
Joint out of limit
机器人是否仍然内八
```

若动作过小但稳定，可逐步提高 `action_scale`：`0.10 -> 0.14`；若仍内八但幅度变小，说明策略趋势本身存在，需要回到训练侧继续约束 hip_roll/yaw、足间距或关节限位奖励。

---

### 5.13 成对降低 5/10 号踝关节 kp/kd

时间：2026-06-29

#### 修改原因

在 `action_scale=0.10`、`clip_actions=3.0`、`act_alpha=0.10`、第二档整体刚度下，机器人能够执行小幅下蹲，但 10 号右踝电机出现持续震动。由于机器人左右对称，10 号 `Right5_ankle_pitch` 对应 5 号 `Left5_ankle_pitch`，因此不单独修改 10 号，而是成对降低 5/10 号踝关节刚度，避免引入左右不对称。

#### 修改文件

```text
src/inference/config/robot.yaml
```

#### 修改前

```yaml
kp:
  [30.0, 30.0, 60.0, 60.0, 60.0,
   30.0, 30.0, 60.0, 60.0, 60.0]
kd:
  [0.8, 0.8, 1.5, 1.5, 1.5,
   0.8, 0.8, 1.5, 1.5, 1.5]
```

#### 修改后

```yaml
kp:
  [30.0, 30.0, 60.0, 60.0, 30.0,
   30.0, 30.0, 60.0, 60.0, 30.0]
kd:
  [0.8, 0.8, 1.5, 1.5, 1.0,
   0.8, 0.8, 1.5, 1.5, 1.0]
```

#### 验证重点

重启后观察：

- 5/10 号踝关节是否仍震动；
- 下蹲幅度是否变化明显；
- 是否出现左右踝不对称；
- 是否继续出现 `Joint out of limit`。

---

### 5.14 新增 inference 日志曲线绘图脚本

时间：2026-06-30

#### 修改文件

```text
scripts/plot_inference_log.py
```

#### 目的

将 `inference_node` 终端调试日志中的关键数据解析为曲线图，方便观察策略输出、目标角、实际反馈、IMU 和误差趋势。

解析字段：

```text
policy_raw[0..9]
cmd_vel[x,y,yaw]
joint_pos[0..9]
imu_quat[w,x,y,z]
gravity_b[x,y,z]
ang_vel[x,y,z]
act_mapped[0..9]
```

生成图像：

```text
policy_raw.png
act_mapped.png
joint_pos.png
joint_tracking.png
joint_error.png
cmd_vel.png
gravity_b.png
ang_vel.png
imu_quat.png
summary.md
```

其中：

- `joint_tracking.png`：逐关节对比 `joint_pos`、`act_mapped` 和 `target-pos` 误差；
- `joint_error.png`：所有关节目标跟踪误差；
- `summary.md`：每个字段的样本数、持续时间、min/max/mean。

#### 使用方法

先导出 screen 终端内容：

```bash
screen -S inference_session -X hardcopy -h /tmp/inference_log.txt
```

再生成图：

```bash
cd /root/roboparty_deploy
python3 scripts/plot_inference_log.py --input /tmp/inference_log.txt
```

默认输出目录：

```text
/root/roboparty_deploy/plots/inference_log_<时间戳>/
```

也可以指定输出目录：

```bash
python3 scripts/plot_inference_log.py \
  --input /tmp/inference_log.txt \
  --output /root/roboparty_deploy/plots/latest
```

#### 验证

已使用当前 `/tmp/inference_log.txt` 试跑通过，输出示例：

```text
/root/roboparty_deploy/plots/inference_log_20260630_161342/
```

生成文件包括：

```text
act_mapped.png
ang_vel.png
cmd_vel.png
gravity_b.png
imu_quat.png
joint_error.png
joint_pos.png
joint_tracking.png
policy_raw.png
summary.md
```

#### 注意

`screen hardcopy` 文件可能含 NUL 字节和终端控制字符，脚本内部会自动清理。若日志中缺少某类字段，则对应图不会生成。

#### 生成图片含义与判读方法

绘图脚本会根据日志中已经打印的字段生成多张 PNG。推荐先看 `joint_tracking.png` 和 `joint_error.png`，再回看 `policy_raw.png`、`act_mapped.png`、`ang_vel.png` 等定位原因。

##### 1. `policy_raw.png`

**含义**：ONNX 策略原始输出，10 条曲线分别对应 10 个关节：

```text
0 L1_hip_yaw
1 L2_hip_roll
2 L3_hip_pitch
3 L4_knee
4 L5_ankle
5 R1_hip_yaw
6 R2_hip_roll
7 R3_hip_pitch
8 R4_knee
9 R5_ankle
```

**怎么看**：

- 如果某些曲线长期超过 `clip_actions`，例如当前 `clip_actions=3.0` 而 `policy_raw[3]≈5~6`、`policy_raw[8]≈-6`，说明策略输出本身已经饱和；
- 长期饱和代表部署端的 `act_mapped` 实际不是完整策略输出，而是被裁剪后的目标；
- 如果策略输出周期性变化明显但实机动作跟不上，可能是 `kp/kd`、`act_alpha`、频率或机械负载问题；
- 如果策略输出本身在 sim2sim 里也导致内八/漂移，则优先回训练侧修改 reward、动作惩罚、关节限位惩罚和姿态约束。

##### 2. `act_mapped.png`

**含义**：策略输出经过裁剪、`action_scale` 缩放、`joint_default_angle` 偏置后的目标关节角。

公式：

```text
act_mapped[i] = clip(policy_raw[i], -clip_actions, clip_actions) * action_scale + joint_default_angle[i]
```

**怎么看**：

- 曲线是控制器希望关节到达的目标角，不是电机实际角；
- 如果曲线长期贴近 `joint_limits`，说明目标太激进，容易触发 out-of-limit；
- 如果曲线长期卡在固定平台值，例如膝关节一直 `0.57/0.63`，通常说明 `policy_raw` 已被 `clip_actions` 截断；
- 若动作幅度太小但稳定，可小幅提高 `action_scale`；若冲限位或抖动，应降低 `action_scale` 或改善策略。

##### 3. `joint_pos.png`

**含义**：驱动反馈的实际关节角，即策略闭环观测里的 `joint_pos`。

**怎么看**：

- 应与 `act_mapped` 有相似趋势，但不一定完全重合；
- 如果 `joint_pos` 长时间完全不变，说明反馈没有实时刷新或电机没有实际运动；
- 如果某个关节靠近红色限位线，说明有 out-of-limit 风险；
- 如果 `joint_pos` 和机器人真实动作不一致，需要回查 DM MIT 模式反馈、`refresh_motor_status()`、零点和反馈转换链路。

##### 4. `joint_tracking.png`

**含义**：每个关节单独一张子图，同时画：

```text
joint_pos      实际反馈角
act_mapped     目标角
target-pos     目标 - 实际误差
joint_limits   红色虚线
joint_default  黑色虚线
```

**怎么看**：

- 这是最重要的综合图；
- 如果 `act_mapped` 与 `joint_pos` 差距持续很大，说明跟踪不足，可能需要更高 `kp`、更合适 `kd` 或更慢的动作目标；
- 如果误差符号持续不变且越来越大，可能是目标过大、负载不足、反馈方向不一致或电机力矩不足；
- 如果 `joint_pos` 围绕目标高频来回，通常是 `kp/kd`、机械间隙、目标抖动或策略振荡导致；
- 如果 `act_mapped` 本身跳变，先看 `policy_raw` 和 `act_alpha`；如果 `act_mapped` 平滑但 `joint_pos` 抖，重点看电机侧。

##### 5. `joint_error.png`

**含义**：所有关节的跟踪误差：

```text
joint_error = act_mapped - joint_pos
```

**怎么看**：

- 用来快速找哪个关节误差最大；
- 如果某个关节误差长期偏正/偏负，说明它一直追不上目标；
- 如果误差正负交替并伴随震荡，说明该关节可能过阻尼/欠阻尼、机械有间隙，或目标本身在抖；
- 如果 5/10 踝关节误差小但仍抖，更多是机械/控制抖动；如果误差大，则是跟踪不足或目标过激。

##### 6. `cmd_vel.png`

**含义**：策略输入中的速度命令：

```text
vx, vy, yaw_rate
```

**怎么看**：

- 静止/下蹲策略测试时应接近 0；
- 如果没有拨手柄但 `cmd_vel` 不为 0，可能是手柄漂移、控制模式切换或死区处理问题；
- 如果 `cmd_vel` 有周期变化，策略可能不是在做零速度动作，而是在响应命令。

##### 7. `gravity_b.png`

**含义**：机体系下重力方向，是策略判断机身姿态的重要输入。

**怎么看**：

- 机器人近似直立时应接近：

```text
x≈0, y≈0, z≈-1
```

- `x/y` 偏离越大，说明机身倾斜越明显；
- 如果静止时 `gravity_b` 明显不对，优先查 IMU 安装方向、四元数顺序和坐标变换；
- 如果运行中 `gravity_b` 开始周期摆动，说明整机姿态已经进入摆动，策略可能在放大这种摆动。

##### 8. `ang_vel.png`

**含义**：IMU 角速度输入：

```text
roll/pitch/yaw angular velocity
```

**怎么看**：

- 静止时应接近 0；
- 如果静止时明显偏离 0，可能是 IMU 漂移、单位错误或解析问题；
- 如果策略运行后 `ang_vel` 出现大幅正负交替，例如 `±0.5 rad/s` 以上，说明整机已进入摆动/振荡；
- 当 `ang_vel` 变大且 `policy_raw` 同时变激进时，通常是策略闭环振荡，不应继续盲目加 `kp`。

##### 9. `imu_quat.png`

**含义**：原始 IMU 四元数：

```text
w, x, y, z
```

**怎么看**：

- 一般不直接用它判断姿态，优先看转换后的 `gravity_b.png`；
- 如果四元数突然跳变或不连续，可能是 IMU 数据异常；
- 如果 `imu_quat` 平滑但 `gravity_b` 异常，检查四元数顺序和坐标变换；
- 如果 `imu_quat` 与 `ang_vel` 同时出现周期震荡，说明机身真实在摆。

##### 推荐判读顺序

每次实机测试后建议按以下顺序看图：

1. `summary.md`：确认样本数、持续时间、是否有异常极值；
2. `joint_tracking.png`：看目标和实际是否跟得上；
3. `joint_error.png`：找误差最大/震荡最大的关节；
4. `policy_raw.png`：看策略是否长期饱和；
5. `act_mapped.png`：看目标是否贴近限位；
6. `ang_vel.png` + `gravity_b.png`：看整机是否进入摆动；
7. `cmd_vel.png`：确认没有意外速度命令；
8. `imu_quat.png`：必要时排查 IMU 原始数据连续性。

##### 常见现象对应判断

| 现象 | 可能原因 | 下一步 |
|---|---|---|
| `policy_raw` 长期超过 `clip_actions` | 策略输出激进/训练侧问题 | 降 `action_scale`，回训练侧加 action penalty / joint limit penalty |
| `act_mapped` 贴近限位 | 目标过大 | 降 `action_scale` 或缩小策略输出 |
| `joint_pos` 追不上 `act_mapped` | 刚度不足/负载大/目标太快 | 先看误差，再小步调 `kp/kd` 或 `act_alpha` |
| `joint_pos` 高频抖 | PD 参数、机械间隙、目标抖动 | 看 `joint_error` 和 `policy_raw`，不要只改单个电机 |
| `ang_vel` 大幅正负交替 | 整机闭环振荡 | 降 `act_alpha` / `action_scale`，检查策略稳定性 |
| `cmd_vel` 非零 | 手柄漂移/控制模式问题 | 查 `/joy`、死区、Y 键模式 |
| `gravity_b` 静止不接近 `[0,0,-1]` | IMU 坐标/安装/四元数问题 | 查 IMU 外参和解析 |


---

### 5.15 新增静态保持稳定性测试脚本

时间：2026-06-30

#### 修改文件

```text
scripts/test_hold_stability.py
```

#### 目的

用于区分“策略输出导致的振动”和“电机/PD/机械本体在静态保持时就会振动”。

脚本不运行 ONNX 策略，而是：

1. 读取指定电机当前反馈位置；
2. 将当前位置作为固定目标；
3. 用 MIT PD 持续保持该目标；
4. 统计保持期间反馈位置的峰峰值和标准差。

如果固定目标不变时电机仍明显振动，说明问题优先在：

- `kp/kd` 参数；
- 电机 MIT 控制稳定性；
- 机械间隙/弹性/摩擦；
- 反馈噪声或量化；
- 电机/关节本体负载状态。

如果静态保持不振，而跑策略才振，则优先看策略目标、`act_mapped`、`act_alpha`、`action_scale` 和闭环姿态。

#### 使用前准备

```bash
cd /root/roboparty_deploy
source /opt/tros/humble/setup.bash
source install/setup.bash
```

确保没有运行推理/控制节点：

```bash
screen -S inference_session -X quit 2>/dev/null || true
```

不要同时运行：

```bash
python3 scripts/listen_motors.py --active
```

#### dry-run

以 10 号右踝为例：

```bash
python3 scripts/test_hold_stability.py --joint 10
```

#### 实际执行

确认机器人支撑/悬空/有人扶持后：

```bash
python3 scripts/test_hold_stability.py --joint 10 --execute
```

默认参数：

```text
DM_J8009_2EC: kp=20, kd=0.2
DM_J10010_2EC: kp=40, kd=0.4
保持 5 秒
命令频率 100 Hz
反馈采样 50 Hz
```

可以指定参数，例如测试当前策略使用的踝关节参数：

```bash
python3 scripts/test_hold_stability.py --joint 10 --kp 30 --kd 0.3 --execute
```

测试一对踝关节建议分别执行：

```bash
python3 scripts/test_hold_stability.py --joint 5 --kp 30 --kd 0.3 --execute
python3 scripts/test_hold_stability.py --joint 10 --kp 30 --kd 0.3 --execute
```

#### 输出判断

脚本会输出：

```text
peak_to_peak=... rad
std=... rad
```

经验判断：

- `peak_to_peak < 0.002 rad`：反馈非常稳；
- `0.002 ~ 0.01 rad`：轻微抖动，可结合肉眼/触感判断；
- `> 0.01 rad`：静态保持已有明显抖动，需要调 PD 或查机械/反馈；
- 若肉眼明显振但反馈峰峰值很小，可能是电机内部高频嗡振、结构共振或反馈采样看不到高频成分。

#### dry-run 验证

已验证：

```bash
python3 -m py_compile scripts/test_hold_stability.py
python3 scripts/test_hold_stability.py --joint 10
```

可正常输出测试计划，不加 `--execute` 不发送控制命令。

---

### 5.16 新策略部署：base height command 对齐、实时打印与当前参数补记

时间：2026-07-01

#### 背景

用户重新训练了一版策略，并说明训练/仿真中已经将任务从“前进动作”修正为“上下蹲动作”，即 `commands[:,0]` 不再表示前进速度，而是表示期望 base height / 质心高度目标。

因此旧部署中：

```text
cmd_vel[x,y,yaw] = [0, 0, 0]
```

对新策略是不正确的。新策略实际需要：

```text
obs[0:3] = [target_base_height, 0, 0]
```

#### 代码修改

##### 1. 增加 base_height 命令模式

修改文件：

```text
src/inference/src/inference_node.hpp
src/inference/src/ros_interface.cpp
src/inference/src/inference_node.cpp
src/inference/config/inference.yaml
install/roboparty_inference/share/roboparty_inference/config/inference.yaml
```

新增配置项：

```yaml
command_mode: "base_height"
base_height_max: 0.913
base_height_min
