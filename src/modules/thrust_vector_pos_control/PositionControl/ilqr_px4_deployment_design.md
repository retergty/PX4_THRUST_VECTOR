# PX4 iLQR 部署代码设计文档

## 1. 文档定位

本文档描述 `thrust_vector_pos_control` 在 PX4 平台中部署 iLQR 速度外环控制器的代码结构、运行链路和接口约束。重点是“代码如何接入 PX4 并在飞控周期内运行”，而不是推导 iLQR 算法、离散模型、代价函数或二次近似。

算法层面的状态定义、离散一阶滞后模型、目标函数和方向变化代价推导见：

- `src/lib/iLQR/iLQR/Documents/Vector_Thrust_UAV_Discrete_Model_First_Order_Lag.md`

本文档对应的主要实现文件为：

- `src/modules/thrust_vector_pos_control/ThrustVectorPositionControl.cpp`
- `src/modules/thrust_vector_pos_control/ThrustVectorPositionControl.hpp`
- `src/modules/thrust_vector_pos_control/PositionControl/PositionControl.cpp`
- `src/modules/thrust_vector_pos_control/PositionControl/PositionControl.hpp`
- `src/modules/thrust_vector_pos_control/PositionControl/ThrustVectorFirstOrderLag.hpp`

## 2. 部署目标

当前实现的部署目标是在 PX4 的位置控制模块框架下，用 iLQR 替换传统速度 PID 的核心加速度命令生成逻辑，同时保留 PX4 原有的状态订阅、参数更新、起飞降落状态机、约束管理、failsafe 和 uORB 输出接口。

在代码层面，iLQR 被部署为 `PositionControl` 内部的速度外环求解器：

1. PX4 模块周期性接收 `vehicle_local_position`、`trajectory_setpoint`、`vehicle_constraints` 等 uORB 数据。
2. `ThrustVectorPositionControl::Run()` 整理飞行状态、起飞状态、速度/加速度限制和姿态模式。
3. `PositionControl::update()` 执行位置外环、iLQR 速度外环和加速度到推力的转换。
4. `ThrustVectorFirstOrderLag.hpp` 中的 iLQR 问题封装负责提供动力学、阶段代价、终端代价、参考轨迹和初始化器。
5. 求解得到的首个控制增量被写回当前命令加速度，再通过 PX4 的 `vehicle_local_position_setpoint` 与 `vehicle_attitude_setpoint` 发布给下游控制链路。

## 3. 模块与编译集成

### 3.1 PX4 模块入口

`thrust_vector_pos_control` 是一个 PX4 module，入口由 `thrust_vector_pos_control_main()` 暴露，模块主体类为 `ThrustVectorPositionControl`。该类继承：

- `ModuleBase<ThrustVectorPositionControl>`：提供 PX4 module 生命周期接口。
- `ModuleParams`：接入 PX4 参数系统。
- `px4::ScheduledWorkItem`：在 `nav_and_controllers` work queue 中周期运行。

模块启动后通过 `vehicle_local_position` callback 驱动控制周期，同时设置 `ScheduleDelayed(100_ms)` 作为备份调度，避免 callback 丢失后控制器完全停摆。

### 3.2 CMake 集成

模块顶层 `CMakeLists.txt` 将 `PositionControl` 和 `Takeoff` 作为子目录加入，并定义 PX4 module：

- `MODULE modules__thrust_vector_pos_control`
- `MAIN thrust_vector_pos_control`
- `DEPENDS ThrustVectorPositionControl ThrustVectorTakeoff controllib geo SlewRate iLQR motion_planning`

`PositionControl/CMakeLists.txt` 将核心控制器封装为库：

- 库名：`ThrustVectorPositionControl`
- 源文件：`ControlMath.cpp`、`PositionControl.cpp`
- 链接：`iLQR`、`mathlib`
- 单元测试：`ControlMathTest.cpp`、`PositionControlTest.cpp`

这意味着 iLQR 不是独立运行的任务，而是作为位置控制模块的内部库依赖被编译进 PX4 固件。

### 3.3 Kconfig 开关

模块由 `MODULES_THRUST_VECTOR_POS_CONTROL` 控制是否编译。受保护板卡环境下还提供 `USER_THRUST_VECTOR_POS_CONTROL`，用于将模块放入 userspace memory。部署到具体机型时，需要在板级配置中启用相应模块选项。

## 4. 运行时数据流

### 4.1 PX4 输入侧

控制周期的主要输入来自 `ThrustVectorPositionControl` 的 uORB 订阅：

- `vehicle_local_position`：位置、速度、航向、EKF reset 信息，是控制周期的主触发源。
- `trajectory_setpoint`：上层 flight task 或 offboard 给出的期望位置、速度、加速度、yaw 和 yaw speed。
- `vehicle_control_mode`：判断是否启用 multicopter position control。
- `vehicle_constraints`：起飞、速度上限等约束。
- `vehicle_land_detected`：地面接触、landed 状态，用于重置 bias 和屏蔽地面阶段控制。
- `hover_thrust_estimate`：可选更新 hover thrust。
- `roll_pitch_setpoint` 与 `thrust_vector_mode`：用于矢量推力模式下指定或释放 roll/pitch 姿态通道。
- `parameter_update`：触发参数刷新。

`set_vehicle_states()` 将 `vehicle_local_position` 转换为 `PositionControlStates`。其中：

- `position` 来自本地位置估计。
- `velocity` 经过 notch filter 和低通滤波。
- `acceleration` 由滤波速度差分得到，作为一阶滞后模型中的当前实际生效加速度估计。
- `yaw` 使用本地位置中的 heading。

### 4.2 控制器内部状态

`PositionControl` 内部维护 iLQR 所需的 9 维状态和 3 维输入：

- `_state.head<3>() = _vel`
- `_state.segment<3>(3) = _acceleration`
- `_state.segment<3>(6) = _input`
- `_input` 表示上一拍已经发送给下游的 NED 命令总加速度。

这三个量分别对应速度、当前实际生效加速度、上一拍命令加速度。每个控制周期求解出的 iLQR 输入是命令加速度增量，代码只取求解轨迹的第一个输入并累加到 `_input`。

### 4.3 PX4 输出侧

控制成功后发布两个 PX4 setpoint：

- `vehicle_local_position_setpoint`：发布内部执行后的 position、velocity、acceleration、thrust setpoint，供其他模块感知控制意图。
- `vehicle_attitude_setpoint`：由 `ControlMath::thrustToAttitude()` 将推力向量和 yaw/roll/pitch setpoint 转换得到，供下游姿态控制执行。

输出链路保持 PX4 现有接口，不直接向电机或执行器发布控制量。iLQR 的输出仍需经过加速度限制、推力转换、姿态 setpoint 和后续姿态/执行器控制链路。

## 5. iLQR 问题封装设计

`ThrustVectorFirstOrderLag.hpp` 采用 header-only 模板封装，目的是在编译期固定状态维度、输入维度和预测步长，减少运行时分配与虚接口之外的额外开销。

### 5.1 命名空间和维度

所有一阶滞后版本实现位于 `thrust_vector_first_order_lag` 命名空间：

- `STATE_DIM = 9`
- `INPUT_DIM = 3`
- `kGravity = 9.80665f`

`ThrustVectorProblem<Scalar, PredictLength>` 将 iLQR core 的 `OptimalControlProblem` 具体化为离散动力学、固定 horizon 的最优控制问题。

### 5.2 动力学适配类

`ThrustVectorDynamicSystem` 继承 `DiscreteSystemDynamicsBase`，向 iLQR core 提供：

- `computeMap()`：状态推进。
- `linearApproximation()`：完整线性化，包含状态推进结果。
- `deviationLinearApproximation()`：偏差线性化。

由于当前部署模型在固定 `dt` 和固定 `alpha` 下是线性的，类内部使用 `PreCompCache` 缓存 `dfdx`、`dfdu` 和相关系数。控制周期内只在 `dt` 或 `alpha` 变化时刷新缓存。

### 5.3 代价类

代码中将部署所需代价拆为三个部分：

- `ThrustVectorDiagonalTrackCost`：阶段跟踪代价，跟踪速度参考，惩罚命令加速度增量，并约束命令加速度靠近参考命令加速度。
- `ThrustDirectionChangeCost`：基于一阶滞后后的实际生效加速度，惩罚相邻时刻推力方向突变。
- `ThrustVectorDiagonalTrackFinalCost`：终端速度跟踪代价。

部署实现默认使用对角权重版本，避免 3 轴小维度问题中不必要的矩阵计算。完整矩阵版本 `ThrustVectorTrackCost` 和 `ThrustVectorTrackFinalCost` 保留为更通用的实现形式。

### 5.4 参考轨迹生成

`ThrustVectorReferenceTrajectoryGenerator` 负责把 PX4 当前周期的速度 setpoint 展开为 iLQR horizon 内的目标轨迹：

- 首次运行时用当前速度初始化内部参考。
- 每个预测步按 `referenceTrajectoryAlpha` 对速度 setpoint 做一阶低通展开。
- 参考状态中的速度维度填入 preview velocity。
- 参考状态中的命令加速度维度填入 bias estimator 给出的命令加速度参考。
- 参考输入默认为零，表示期望命令增量尽量小。

这部分属于部署适配逻辑：它把 PX4 单点 setpoint 转换成 iLQR solver 需要的目标轨迹数组，而不改变算法文档中的模型定义。

### 5.5 OCP 和 Solver 组合

`ThrustVectorOptimalControlProblem` 组装动力学、阶段代价和终端代价。`ThrustVectorILQR` 在此基础上持有：

- `HoverInitializer`：用于给 solver 提供初始输入和 rollout。
- `iLQR<Descriptor_t> solver_`：实际求解器。
- `ThrustVectorReferenceTrajectoryGenerator`：目标轨迹生成器。

`createThrustVectorFirstOrderLagProblem()` 提供工厂函数，但当前 `PositionControl` 直接构造 `ThrustVectorILQR<float, kPredictLength>`。

## 6. PositionControl 集成逻辑

### 6.1 构造期配置

`PositionControl` 构造函数中完成 iLQR 设置：

- 预测步长：`kPredictLength = 25`
- 时间步长：`kTimeStep = 1 / 125`
- solver 最大迭代次数：`maxNumIterations = 3`
- 一阶滞后系数：`alpha = 0.3`
- 参考轨迹低通：`{0.4, 0.4, 0.6}`
- 阶段权重：`Q`、`R`、`Ra`
- 终端权重：`Qf`
- 推力方向变化权重：`weight`

当前这些 iLQR 参数是在 C++ 构造函数中固定配置的，不通过 PX4 参数系统在线更新。PX4 参数系统主要负责位置环增益、速度/加速度/推力限制、hover thrust、bias estimator 和起飞降落相关参数。

### 6.2 每周期执行顺序

`PositionControl::update()` 的执行顺序为：

1. `_inputValid()` 检查 setpoint 和状态有效性。
2. `_positionControl()` 将 position setpoint 转换为 velocity setpoint，并叠加 feed-forward velocity。
3. `_velocityControl(dt)` 运行 iLQR，生成新的命令加速度。
4. `_accelerationControl()` 将加速度 setpoint 转换成归一化推力向量。
5. 填充 yaw/yaw speed 默认值。

其中 iLQR 只位于 `_velocityControl()` 内。位置环仍是 PX4 风格的 P 外环，iLQR 接收的是速度目标而不是位置目标。

### 6.3 iLQR 调用步骤

`_velocityControl()` 中的 iLQR 调用流程为：

1. 计算速度误差，用于更新低频加速度 bias estimator。
2. 根据当前命令加速度是否接近限制，决定是否冻结某些轴的 bias 更新。
3. 调用 `setDesireTrajectory(_time, _vel_sp, _vel, accel_bias_estimator_.bias())` 生成 horizon 参考轨迹。
4. 写入 iLQR 初始状态：
   - 当前速度。
   - 当前实际生效加速度估计。
   - 上一拍命令加速度。
5. 调用 `_ilqr->solver().run(_time, _state)`。
6. 读取 `primalSolution.inputTrajectory_.front()` 作为本周期命令加速度增量。
7. 将增量累加到 `_input`。
8. 对 `_input` 做水平和竖直加速度限制。
9. 将 `_input` 叠加到 `_acc_sp`，形成最终加速度 setpoint。

这种方式采用 receding horizon 控制：每周期只执行优化序列中的第一拍，其余轨迹只用于预测。

### 6.4 加速度到推力/姿态

`_accelerationControl()` 将 NED 加速度 setpoint 转为归一化推力：

1. 在 z 轴加入重力项。
2. 按 hover thrust 与重力常数比例换算为归一化推力。
3. 对最小推力做基本限制。

随后 `getAttitudeSetpoint()` 调用 `ControlMath::thrustToAttitude()`：

- 普通模式下，由推力向量和 yaw 生成姿态。
- Pitch/Roll/ThrustVector 模式下，按外部 roll/pitch setpoint 约束姿态，再计算 body thrust。

因此 iLQR 部署层输出的是“期望总加速度/推力向量”，不是舵机角、关节角或电机 PWM。

## 7. PX4 状态机与保护逻辑

### 7.1 起飞和地面阶段

`ThrustVectorPositionControl::Run()` 保留 PX4 takeoff handling：

- 未起飞或飞行中检测到 ground contact 时，清空 setpoint 并设置向下加速度，避免地面阶段产生推力修正。
- 地面阶段重置 acceleration bias estimator，避免把地面约束学习成空中扰动。
- 起飞 ramp 阶段限制上升速度、倾角和最小推力。
- 未进入 flying 状态时，hover thrust 使用参数值而不是估计值的动态更新。

iLQR 求解器本身不直接判断 landed 或 takeoff 状态，这些保护逻辑在 PX4 模块外层完成。

### 7.2 输入有效性与 failsafe

`PositionControl::_inputValid()` 要求每个轴至少有 position、velocity 或 acceleration setpoint，并要求对应估计状态有效。若 `update()` 失败：

1. 外层生成 failsafe setpoint。
2. 重置约束。
3. 重新设置速度和加速度限制。
4. 再次调用 `PositionControl::update()`。

这保证 iLQR 输入无效时，控制器仍沿用 PX4 的安全降级路径。

### 7.3 EKF reset 处理

`adjustSetpointForEKFResets()` 根据 `vehicle_local_position` 中的位置、速度和航向 reset counter 修正当前 setpoint，避免估计坐标跳变后 setpoint 与状态产生虚假误差。

iLQR 不直接处理 EKF reset。部署层在进入 `PositionControl` 前修正 setpoint，使求解器看到的是已经补偿过的目标。

### 7.4 Bias estimator 的部署角色

低频加速度 bias estimator 用于给 iLQR 的命令加速度参考提供慢变量补偿。它不属于 iLQR solver 本身，而是 PX4 部署层为了处理稳态扰动、模型偏差和一阶滞后误差加入的外部状态。

相关设计见：

- `src/modules/thrust_vector_pos_control/PositionControl/bias_estimator_design.md`

当前实现中，bias 会在以下场景被管理：

- 地面或非 position control 阶段重置。
- 水平轴不受控时重置 XY bias。
- 命令加速度接近限制且误差继续要求向饱和方向增加时，冻结对应轴。
- 参数更新时刷新积分、泄漏、低通和限幅配置。

## 8. 与算法设计文档的边界

本文档只描述部署和代码组织，不重复以下内容：

- 状态空间模型推导。
- 一阶低通离散化公式。
- iLQR/DDP backward pass 与 forward rollout 推导。
- 代价函数 Hessian/Jacobian 计算。
- 推力方向变化代价的 Gauss-Newton 近似。
- 权重参数的算法含义推导。

算法文档回答“为什么这个模型和代价这样定义”。本文档回答“这些模型和代价在 PX4 中由哪些类承载、每个控制周期如何被调用、输入输出如何接入 uORB、哪些 PX4 状态机会包住 solver”。

## 9. 当前实现约束与注意事项

### 9.1 固定 horizon 与固定 solver 配置

`kPredictLength`、`kTimeStep`、`maxNumIterations` 和 iLQR 权重目前在编译期或构造期固定。若后续需要在线调参，需要增加 PX4 参数并在 `parameters_update()` 中同步到 `PositionControl` 或重建 solver。

### 9.2 当前有效加速度估计来自速度差分

iLQR 初始状态中的 `a_eff` 当前由滤波速度差分得到。这种方式部署简单，不依赖额外执行器反馈，但会受到速度滤波、采样周期和估计噪声影响。相关滤波参数由 `MPC_VEL_LP`、`MPC_VEL_NF_FRQ`、`MPC_VEL_NF_BW`、`MPC_VELD_LP` 管理。

### 9.3 Solver 输出后仍需 PX4 限幅

iLQR 输出的命令增量累加到 `_input` 后，代码继续执行水平加速度模长限制和竖直加速度限制。实际可执行性由 PX4 外层约束兜底，而不是完全依赖 solver 内部约束。

当前 `ThrustVectorProblem` 使用 `ConstraintConfig<>`，即 iLQR 问题本身未注册显式约束。部署层必须保留输出限幅、failsafe 和起飞状态机。

### 9.4 姿态与推力约束在部署链路后段处理

iLQR 不直接处理 roll/pitch/yaw 姿态执行细节。姿态生成由 `ControlMath::thrustToAttitude()` 完成，并受 `thrust_vector_mode`、`roll_pitch_setpoint` 和下游姿态控制器影响。

在阅读代码或调试飞行效果时，需要区分：

- iLQR 生成的是 NED 命令总加速度。
- `PositionControl` 将命令总加速度换算成推力向量。
- `ControlMath` 将推力向量和姿态约束换算成 attitude setpoint。
- 下游姿态/执行器模块负责真正执行。

### 9.5 `ThrustVector.hpp` 与当前一阶滞后实现

`PositionControl.hpp` 当前包含的是 `ThrustVectorFirstOrderLag.hpp`，因此活跃实现是一阶滞后版本。`ThrustVector.hpp` 可视为历史或备用实现，阅读部署链路时应以 `ThrustVectorFirstOrderLag.hpp` 为准。

## 10. 调试与验证建议

代码级验证建议分三层进行：

1. 编译层：确认 `ThrustVectorPositionControl` 能链接 `iLQR`，模块能随目标固件编译。
2. 单元测试层：运行 `PositionControlTest` 与 `ControlMathTest`，确认基础 setpoint、推力和姿态转换行为未被破坏。
3. 运行层：在 SITL 或实机日志中观察 `vehicle_local_position_setpoint.acceleration`、`vehicle_local_position_setpoint.thrust`、`vehicle_attitude_setpoint`、速度误差、bias 估计和 iLQR 平均迭代次数。

`PositionControl::print_status()` 当前会输出 iLQR 对象大小和平均迭代次数，可用于确认 solver 是否在运行以及迭代负载是否符合预期。

## 11. 后续扩展方向

若继续完善 PX4 部署，可优先考虑以下方向：

- 将 iLQR 权重、`alpha`、参考轨迹低通和最大迭代次数接入 PX4 参数系统。
- 为 `ThrustVectorFirstOrderLag.hpp` 增加针对动力学缓存、参考轨迹生成和首拍输出逻辑的单元测试。
- 在日志中增加 iLQR 求解状态、首拍增量、命令加速度、bias、有效加速度估计等观测量。
- 明确输出限幅与 solver 内部约束的责任边界，必要时将部分约束前移到 OCP 层。
- 评估当前速度差分加速度估计是否需要替换为 IMU/EKF 融合后的更稳定 `a_eff` 估计。
