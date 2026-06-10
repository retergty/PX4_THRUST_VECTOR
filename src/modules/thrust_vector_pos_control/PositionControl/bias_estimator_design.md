# Low-Frequency Acceleration Bias Estimator Design

## 1. 设计目标

在当前增量式 MPC 中，优化输入为命令加速度增量。记命令加速度为 $\mathbf{a}_{cmd}$，上一拍命令加速度为 $\mathbf{a}_{cmd,prev}$，MPC 输入为 $\Delta \mathbf{a}_{cmd}$，则系统内部满足：

$$
\mathbf{a}_{cmd} = \mathbf{a}_{cmd,prev} + \Delta \mathbf{a}_{cmd}
$$

如果代价函数只惩罚 $\Delta \mathbf{a}_{cmd}$，则 $\mathbf{a}_{cmd}$ 会表现为无泄漏积分状态。该结构有利于命令连续性，但在一阶滞后、执行器饱和或模型不匹配时，容易造成命令加速度长期积累，进而引起过冲和低频振荡。

如果直接惩罚 $\mathbf{a}_{cmd}$ 靠近零，即：

$$
J_a = \frac{1}{2}\mathbf{a}_{cmd}^{T}R_a\mathbf{a}_{cmd}
$$

则可以抑制命令漂移，但会削弱系统对常值扰动和模型偏差的补偿能力。因此，建议引入低频加速度偏置估计器 $\mathbf{a}_{bias}$，构造命令加速度参考：

$$
\mathbf{a}_{cmd,ref} = \mathbf{a}_{ff} + \mathbf{a}_{bias}
$$

并将命令加速度代价改为：

$$
J_a = \frac{1}{2}
(\mathbf{a}_{cmd} - \mathbf{a}_{cmd,ref})^{T}
R_a
(\mathbf{a}_{cmd} - \mathbf{a}_{cmd,ref})
$$

这样，$R_a$ 的作用不再是强制命令加速度回到零，而是约束命令加速度不要偏离“轨迹前馈 + 低频扰动补偿”过多。

## 2. 与 PX4 标准四旋翼控制逻辑的关系

PX4 标准 `mc_pos_control` 中的速度积分器并非在所有飞行阶段持续工作，而是根据飞行状态、控制轴有效性和执行器饱和情况进行管理。其核心原则包括：

- 未起飞、未飞行或存在地面接触时，重置积分器，避免将地面约束或桨盘未完全建立推力的过程误认为空中扰动。
- 起飞 ramp 阶段限制上升速度、倾角和最小推力，使控制器从地面模型平滑过渡到空中模型。
- 只有在飞行状态稳定、控制轴有效时，才允许积分器正常工作。
- 推力饱和时执行 anti-windup，避免积分器继续向不可实现的方向累积。
- hover thrust 估计值变化时，通过调整积分项保持输出推力连续。
- 某个轴不受控时，重置该轴积分器，避免重新接管时出现过补偿。

MPC 中的低频 bias estimator 应遵循相同原则。它不应在模型显著失配的阶段学习偏置，而应仅在“飞行状态可信、控制轴有效、执行器未严重饱和”的条件下更新。

### 2.1 PX4 代码依据

未起飞或存在地面接触时，PX4 会清空 setpoint 并重置速度积分器。该逻辑避免控制器在地面约束下学习错误积分量：

```cpp
// src/modules/mc_pos_control/MulticopterPositionControl.cpp
const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

if (not_taken_off || flying_but_ground_contact) {
	// we are not flying yet and need to avoid any corrections
	_setpoint = PositionControl::empty_trajectory_setpoint;
	_setpoint.timestamp = vehicle_local_position.timestamp_sample;
	Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

	// prevent any integrator windup
	_control.resetIntegral();
}
```

起飞阶段，PX4 通过 takeoff state 限制倾角、上升速度和最小推力，使控制器从地面状态平滑进入飞行状态：

```cpp
// src/modules/mc_pos_control/MulticopterPositionControl.cpp
// limit tilt during takeoff ramupup
const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
			     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

const float speed_up = _takeoff.updateRamp(dt,
		       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());

// Allow ramping from zero thrust on takeoff
const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

_control.setVelocityLimits(
	max_speed_xy,
	math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
	math::max(speed_down, 0.f));
```

速度积分器只在速度控制流程中更新，并且更新前会先执行垂向限幅、垂向 anti-windup 和水平 tracking anti-windup：

```cpp
// src/modules/mc_pos_control/PositionControl/PositionControl.cpp
// Constrain vertical velocity integral
_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

// Integrator anti-windup in vertical direction
if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
	vel_error(2) = 0.f;
}

// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
	const float arw_gain = 2.f / _gain_vel_p(0);
	const Vector2f acc_sp_xy = _acc_sp.xy();

	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
}

// Update integral part of velocity control
_vel_int += vel_error.emult(_gain_vel_i) * dt;
```

当 hover thrust 估计值变化时，PX4 会调整积分项，使同一加速度 setpoint 对应的输出推力保持连续。这说明积分器不仅是误差累积项，也承担了低频模型偏差补偿的作用：

```cpp
// src/modules/mc_pos_control/PositionControl/PositionControl.cpp
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}
```

当水平轴不受控时，PX4 会重置水平积分器，避免重新接管时使用过期的低频补偿：

```cpp
// src/modules/mc_pos_control/MulticopterPositionControl.cpp
if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
	// Horizontal velocity is not controlled, reset the integrators to avoid
	// over-corrections when starting again.
	_control.resetIntegralXY();
}
```

## 3. 状态定义

建议新增低频加速度偏置状态：

$$
\mathbf{a}_{bias} \in \mathbb{R}^{3}
$$

其坐标系与 MPC 命令加速度一致，均为 NED 坐标系，单位为 $\mathrm{m/s^2}$。

$\mathbf{a}_{bias}$ 表示为了抵消慢变扰动或模型偏差而需要长期保留的命令加速度补偿。典型来源包括：

- 标称 hover thrust 与真实悬停推力不一致；
- 重心不对称导致的姿态或推力偏置；
- 稳态风扰；
- 推力模型误差；
- 机体安装误差或矢量推力机构零位偏差。

## 4. Bias 更新律

令速度误差为：

$$
\mathbf{e}_v = \mathbf{v}_{sp} - \mathbf{v}
$$

基础连续形式可写为：

$$
\dot{\mathbf{a}}_{bias}
= K_{i,bias}\mathbf{e}_v
- \lambda \mathbf{a}_{bias}
$$

离散实现为：

$$
\mathbf{a}_{bias,k+1}
= \mathbf{a}_{bias,k}
+ \left(
K_{i,bias}\mathbf{e}_{v,k}
- \lambda \mathbf{a}_{bias,k}
\right)\Delta t
$$

其中：

- $K_{i,bias}$ 为低频积分增益，应显著小于主控制回路带宽。
- $\lambda$ 为可选泄漏系数，用于防止错误偏置长期保持。
- $\mathbf{e}_v$ 建议经过限幅或低通滤波，避免估计器学习高频振荡。

推荐实现形式为：

$$
\mathbf{e}_{v,f} = LPF(\mathbf{v}_{sp} - \mathbf{v})
$$

$$
\mathbf{a}_{bias,k+1}
= sat\left[
rate\_limit\left(
\mathbf{a}_{bias,k}
+ (K_{i,bias}\mathbf{e}_{v,f}
- \lambda\mathbf{a}_{bias,k})\Delta t
\right)
\right]
$$

其中 $sat(\cdot)$ 表示按轴限幅，`rate_limit` 表示可选变化率限制。

## 5. 启用条件

仅当以下条件同时满足时，才允许更新 $\mathbf{a}_{bias}$：

- vehicle 已 armed；
- takeoff state 已进入 `flight`；
- land detector 未检测到 `landed`；
- vehicle 未处于 `ground_contact`；
- 当前轴速度估计有效；
- 当前轴速度 setpoint 有效；
- 当前轴由位置/速度控制器接管；
- 推力或加速度命令未在该轴严重饱和；
- MPC 求解成功，且输出为有限值。

若上述条件不满足，应根据状态选择 freeze 或 reset，而不应继续积分。

## 6. Reset 与 Freeze 策略

建议采用以下状态机策略：

| 条件                            | 处理                           |
| ------------------------------- | ------------------------------ |
| disarmed                        | reset $\mathbf{a}_{bias}$      |
| landed                          | reset $\mathbf{a}_{bias}$      |
| not taken off                   | reset $\mathbf{a}_{bias}$      |
| ground contact                  | reset $\mathbf{a}_{bias}$      |
| takeoff ramp                    | freeze $\mathbf{a}_{bias}$     |
| stable flight                   | update $\mathbf{a}_{bias}$     |
| landing descend, no touch-down  | freeze 或 very slow update     |
| axis uncontrolled               | reset 对应轴 bias              |
| estimator reset / velocity jump | freeze 一段时间或 reset 对应轴 |

起飞前必须 reset，因为此时速度误差包含地面约束、推力未建立、桨盘气动变化等因素，不能代表空中稳态扰动。

降落触地后也必须 reset 或 freeze，因为地面支撑力会改变垂向动力学。若此时继续学习，估计器可能将地面反力误识别为需要补偿的外部扰动。

## 7. Anti-Windup 设计

当控制输出受限时，bias estimator 不应继续向更饱和的方向积分。

垂直方向可采用如下逻辑：

$$
\text{if thrust at max and } e_{v,z} \text{ requests more upward acceleration, freeze } a_{bias,z}
$$

$$
\text{if thrust at min and } e_{v,z} \text{ requests more downward acceleration, freeze } a_{bias,z}
$$

水平方向可采用更简单的第一版策略：

$$
\|\mathbf{a}_{cmd,xy}\| \ge a_{xy,max}
\quad \Rightarrow \quad
freeze(\mathbf{a}_{bias,xy})
$$

后续可扩展为 tracking anti-windup，将不可实现的加速度误差从积分输入中扣除。第一版建议优先采用按轴 freeze，因为行为更可预测，调试风险更低。

## 8. 与 MPC Cost 的接口

当前命令加速度定义为：

$$
\mathbf{a}_{cmd}
= \mathbf{a}_{cmd,prev} + \Delta\mathbf{a}_{cmd}
$$

加入 bias reference 后，命令加速度幅值代价应写为：

$$
J_a
= \frac{1}{2}
(\mathbf{a}_{cmd} - \mathbf{a}_{cmd,ref})^T
R_a
(\mathbf{a}_{cmd} - \mathbf{a}_{cmd,ref})
$$

若使用对角权重 $R_a = diag(r_{a,x}, r_{a,y}, r_{a,z})$，则：

$$
J_a
= \frac{1}{2}
\sum_{i \in \{x,y,z\}}
r_{a,i}
(a_{cmd,i} - a_{cmd,ref,i})^2
$$

各权重含义如下：

- $R$：惩罚 $\Delta\mathbf{a}_{cmd}$，主要决定命令变化率和平滑性。
- $R_a$：惩罚 $\mathbf{a}_{cmd}$ 偏离低频参考，主要抑制命令漂移和过大幅值。
- $\mathbf{a}_{bias}$：提供常值扰动补偿，避免 $R_a$ 削弱稳态控制能力。

## 9. 前馈加速度

若参考轨迹只生成速度参考，可由参考速度差分得到前馈加速度：

$$
\mathbf{a}_{ff,k}
= \frac{\mathbf{v}_{ref,k+1} - \mathbf{v}_{ref,k}}{\Delta t}
$$

若上层已提供有效加速度 setpoint，则可使用：

$$
\mathbf{a}_{ff} = \mathbf{a}_{sp}
$$

第一版实现可以先采用：

$$
\mathbf{a}_{cmd,ref} = \mathbf{a}_{bias}
$$

在 bias 更新逻辑验证稳定后，再加入轨迹前馈：

$$
\mathbf{a}_{cmd,ref} = \mathbf{a}_{ff} + \mathbf{a}_{bias}
$$

## 10. 参数建议

起始参数建议如下：

| 参数                             | 建议范围                          |
| -------------------------------- | --------------------------------- |
| $K_{i,bias,xy}$                  | $0.05 \sim 0.20\ \mathrm{s^{-1}}$ |
| $K_{i,bias,z}$                   | $0.05 \sim 0.15\ \mathrm{s^{-1}}$ |
| $\lambda$                        | $0.01 \sim 0.05\ \mathrm{s^{-1}}$ |
| $\|\mathbf{a}_{bias,xy}\|_{max}$ | $0.5 \sim 2.0\ \mathrm{m/s^2}$    |
| $                                | a_{bias,z}                        | _{max}$ | $0.5 \sim 2.0\ \mathrm{m/s^2}$ |

若仿真中仍存在低频稳态误差，可适当增大 $K_{i,bias}$ 或 bias 限幅。

若起飞、降落或速度阶跃时 bias 学到错误补偿，应减小 $K_{i,bias}$，增加误差滤波，或延长 freeze 条件。

若飞行中响应明显变钝，应减小 $R_a$，减小 $\lambda$，或降低 bias 对命令参考的作用强度。

## 11. 推荐实现步骤

1. 新增 $\mathbf{a}_{bias}$ 成员变量，并提供 reset、freeze、update 接口。
2. 在外层模块中根据 armed、landed、ground contact 和 takeoff state 生成 bias estimator enable 状态。
3. 在 `flight` 状态且控制轴有效时，使用低通后的速度误差慢速更新 $\mathbf{a}_{bias}$。
4. 对 $\mathbf{a}_{bias}$ 做按轴限幅和可选 rate limit。
5. 将 $\mathbf{a}_{cmd,ref}$ 写入 target trajectory，或单独传入 command acceleration cost。
6. 将 `CommandAccelerationDiagonalCost` 从惩罚 $\mathbf{a}_{cmd}$ 改为惩罚 $\mathbf{a}_{cmd} - \mathbf{a}_{cmd,ref}$。
7. 在起飞、降落、切模式和 estimator reset 时验证 bias 是否正确 freeze 或 reset。
8. 先在 SITL 中测试 hover、速度阶跃、慢速巡航和降落触地，再进行小范围真机验证。

## 12. 第一版实现建议

为降低实现风险，第一版建议采用保守方案：

$$
\mathbf{a}_{cmd,ref} = \mathbf{a}_{bias}
$$

并采用以下约束：

- takeoff ramp 阶段 freeze；
- ground contact 和 landed 状态 reset；
- 饱和时 freeze 对应轴；
- $R_a$ 初始取 $R$ 的 $5\% \sim 20\%$；
- $K_{i,bias}$ 使用较小值；
- bias 更新频率与主控制循环一致，但有效带宽应显著低于 MPC 闭环带宽。

该方案保留 MPC 的主要动态控制能力，同时让 bias estimator 只学习低频常值补偿，避免在起飞、降落和执行器饱和等模型不匹配阶段引入错误积分。
# Low-Frequency Acceleration Bias Estimator Design

## 目标

在增量式 MPC 中，`delta_a_cmd` 是优化输入，实际命令加速度通过积分得到：

```text
a_cmd = a_cmd_prev + delta_a_cmd
```

如果只惩罚 `delta_a_cmd`，`a_cmd` 容易成为无泄漏积分状态，导致过冲和抖动。如果直接惩罚 `a_cmd -> 0`，又会削弱对常值扰动的补偿能力。

因此引入一个低频加速度偏置估计器 `a_bias`，作为命令加速度参考的一部分：

```text
a_cmd_ref = a_ff + a_bias
```

然后将命令加速度幅值代价从：

```text
||a_cmd||_Ra^2
```

改为：

```text
||a_cmd - a_cmd_ref||_Ra^2
```

这样 `Ra` 不再强行把命令拉回零，而是把命令拉向“前馈加速度 + 慢速扰动补偿”。

## 参考 PX4 标准四旋翼逻辑

PX4 标准 `mc_pos_control` 的速度积分器不是在所有阶段自由积分，而是按飞行阶段和执行器饱和状态管理：

- 未起飞、未飞行、地面接触时，清空积分器，避免把地面约束误认为飞行扰动。
- 起飞 ramp 阶段限制速度和倾角，让推力从地面状态平滑过渡到空中模型。
- 飞行中才正常积分。
- 推力饱和时执行 anti-windup，避免积分器继续往不可实现方向累积。
- hover thrust 估计变化时，调整积分项，使输出推力不因参数更新产生突变。
- 某个轴不受控时，重置对应轴积分器，避免重新接管时过补偿。

MPC 的 bias estimator 应遵循同样原则：只在“模型基本可信、控制轴有效、执行器未严重饱和”的条件下学习低频偏置。

## 状态定义

建议新增状态：

```text
a_bias: NED 加速度偏置估计，单位 m/s^2
```

它表示为了抵消慢变扰动或模型偏差，需要长期保留的命令加速度补偿。

典型来源包括：

- hover thrust 标称值与真实悬停推力不一致；
- 重心不对称导致的姿态/推力偏置；
- 风扰；
- 推力模型误差；
- 机体安装或矢量推力机构偏置。

## 更新律

基础形式：

```text
e_v = v_sp - v
a_bias_dot = Ki_bias * e_v - leak * a_bias
a_bias = a_bias + a_bias_dot * dt
```

其中：

- `Ki_bias` 是很小的低频积分增益。
- `leak` 是可选泄漏项，防止 bias 永久保留错误估计。
- `e_v` 建议使用限幅或低通后的速度误差，避免学习高频振荡。

推荐实现形式：

```text
e_v_filtered = lowpass(v_sp - v)
a_bias += (Ki_bias * e_v_filtered - leak * a_bias) * dt
a_bias = constrain_axiswise(a_bias)
a_bias = rate_limit(a_bias)
```

## 启用条件

只有满足以下条件时才允许更新 `a_bias`：

- vehicle 已 armed；
- takeoff state 已进入 `flight`；
- land detector 未检测到 `landed`；
- 未处于 `ground_contact`；
- 当前轴速度估计有效；
- 当前轴速度 setpoint 有效；
- 当前轴由位置/速度控制器接管；
- 推力或加速度命令未在该轴严重饱和；
- MPC 求解成功，输出有限值。

如果条件不满足，应 freeze 或 reset。

## Reset 与 Freeze 策略

建议策略：

```text
disarmed: reset a_bias
landed: reset a_bias
not_taken_off: reset a_bias
ground_contact: reset a_bias
takeoff ramp: freeze a_bias
flight: update a_bias
landing descend but not touched: freeze 或 very slow update
axis uncontrolled: reset 对应轴 bias
estimator reset / velocity jump: freeze 一小段时间或 reset 对应轴
```

起飞前必须 reset，因为此时速度误差包含地面约束和推力未建立的影响，不是空中扰动。

降落触地后也必须 reset 或 freeze，因为此时地面支撑力会让估计器学到错误的向下/向上补偿。

## Anti-Windup

当输出受限时，不应让 bias 继续往更饱和方向积分。

垂直方向示例：

```text
if thrust at max and e_v requests more upward acceleration:
    do not integrate upward bias

if thrust at min and e_v requests more downward acceleration:
    do not integrate downward bias
```

水平方向示例：

```text
if horizontal acceleration saturated:
    project error onto unsaturated direction
    or freeze horizontal bias update
```

简单可靠的第一版可以采用：

```text
if acceleration command saturated:
    freeze affected axis bias
```

后续再扩展为 tracking anti-windup。

## 与 MPC Cost 的关系

当前命令加速度代价为：

```text
a_cmd = a_cmd_prev + delta_a_cmd
J_a = 0.5 * (a_cmd)^T * Ra * (a_cmd)
```

加入 bias reference 后应改为：

```text
a_cmd_ref = a_ff + a_bias
J_a = 0.5 * (a_cmd - a_cmd_ref)^T * Ra * (a_cmd - a_cmd_ref)
```

其中：

- `R` 继续惩罚 `delta_a_cmd`，控制命令变化率和平滑性。
- `Ra` 惩罚 `a_cmd` 偏离低频参考，控制命令幅值和漂移。
- `a_bias` 提供常值扰动补偿，避免 `Ra` 削弱稳态控制能力。

## 前馈加速度 `a_ff`

如果当前参考轨迹只生成速度参考，可用参考速度差分得到低频前馈：

```text
a_ff[k] = (v_ref[k + 1] - v_ref[k]) / dt
```

如果上层已经提供有效 `_acc_sp`，可将其作为前馈的一部分：

```text
a_ff = valid_acc_sp
```

第一版可以先只使用：

```text
a_cmd_ref = a_bias
```

等 bias 逻辑稳定后，再加入轨迹前馈。

## 参数建议

起始调参建议：

```text
Ki_bias_xy: 0.05 ~ 0.2 1/s
Ki_bias_z:  0.05 ~ 0.15 1/s
leak:       0.01 ~ 0.05 1/s
a_bias_xy_limit: 0.5 ~ 2.0 m/s^2
a_bias_z_limit:  0.5 ~ 2.0 m/s^2
```

如果仿真中仍有低频稳态误差，适当增大 `Ki_bias` 或 limit。

如果起飞/降落或速度阶跃时 bias 学错，减小 `Ki_bias`、增大滤波、延长 freeze 条件。

如果飞行中响应变钝，减小 `Ra` 或减小 bias 的 leak。

## 推荐实现步骤

1. 新增 `a_bias` 成员变量和 reset/freeze/update 接口。
2. 在外层模块中根据 armed、landed、ground_contact、takeoff state 生成 bias estimator enable 状态。
3. 在 `flight` 状态且控制轴有效时，用速度误差慢速更新 `a_bias`。
4. 对 `a_bias` 做轴向限幅和可选 rate limit。
5. 将 `a_cmd_ref` 写入 target trajectory 或单独传入 command acceleration cost。
6. 将 `CommandAccelerationDiagonalCost` 从惩罚 `a_cmd` 改为惩罚 `a_cmd - a_cmd_ref`。
7. 起飞、降落、切模式、估计器 reset 时验证 bias 是否正确 freeze/reset。
8. 先在 SITL 中测试 hover、阶跃速度、慢速巡航、降落触地，再上真机小范围验证。

## 第一版建议

为了降低风险，第一版建议：

```text
a_cmd_ref = a_bias
takeoff ramp freeze
ground_contact reset
landed reset
饱和时 freeze 对应轴
Ra 取 R 的 5% ~ 20%
Ki_bias 使用很小值
```

这样可以保留 MPC 的主要动态控制能力，同时让 bias 只学习低频常值补偿，避免在模型不匹配阶段引入错误积分。
