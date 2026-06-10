# PositionControl 文档路由

本文档是 `src/modules/thrust_vector_pos_control/PositionControl` 的阅读入口，用于说明当前 iLQR 相关设计文档的位置、关注范围和推荐阅读顺序。

## 推荐阅读顺序

1. 先读 `src/lib/iLQR/iLQR/Documents/Vector_Thrust_UAV_Discrete_Model_First_Order_Lag.md`，理解一阶滞后增量式矢量推力模型和代价函数的算法定义。
2. 再读 `src/modules/thrust_vector_pos_control/PositionControl/ilqr_px4_deployment_design.md`，理解上述算法模型如何部署进 PX4 的 `thrust_vector_pos_control` 模块。
3. 最后读 `src/modules/thrust_vector_pos_control/PositionControl/bias_estimator_design.md`，理解部署层为什么需要低频加速度 bias estimator，以及它如何和 iLQR 命令加速度参考相连。

## 文档索引

### iLQR 算法模型文档

路径：

`src/lib/iLQR/iLQR/Documents/Vector_Thrust_UAV_Discrete_Model_First_Order_Lag.md`

该文档位于 iLQR 库的 `Documents` 目录下，是算法设计文档。它主要解释一阶低通执行器离散模型本身，包括状态与输入定义、离散动力学、线性离散形式、目标函数、推力方向变化代价、Gauss-Newton 二次近似和参数选择建议。

阅读该文档时应关注“控制问题是什么”和“代价函数为什么这样定义”。它不负责解释 PX4 module 如何调度、uORB 如何接入、参数如何更新、failsafe 如何处理。

### PX4 iLQR 部署代码设计文档

路径：

`src/modules/thrust_vector_pos_control/PositionControl/ilqr_px4_deployment_design.md`

该文档位于 PX4 `thrust_vector_pos_control` 模块的 `PositionControl` 目录下，是代码部署设计文档。它主要解释 iLQR 如何作为 PX4 位置控制模块内部的速度外环求解器运行，包括模块入口、CMake/Kconfig 集成、uORB 输入输出、`ThrustVectorPositionControl::Run()` 调度、`PositionControl::_velocityControl()` 中的 solver 调用、加速度到推力/姿态 setpoint 的转换，以及起飞状态机、failsafe、EKF reset、输出限幅等 PX4 部署层保护逻辑。

阅读该文档时应关注“算法模型如何被放进 PX4 控制周期里运行”。它刻意不重复模型公式和 iLQR 推导。

### 低频加速度 Bias Estimator 设计文档

路径：

`src/modules/thrust_vector_pos_control/PositionControl/bias_estimator_design.md`

该文档同样位于 PX4 `PositionControl` 目录下，是部署层补偿逻辑的设计文档。它主要解释为什么增量式 iLQR 需要低频加速度 bias estimator，如何用速度误差估计慢变命令加速度偏置，如何设计积分、泄漏、低通、限幅、reset、freeze 和 anti-windup 策略，以及它如何通过命令加速度参考影响 `Ra` 代价项。

阅读该文档时应关注“PX4 部署环境下如何处理稳态扰动、模型偏差和执行器饱和”。它是 iLQR solver 外部的工程补偿逻辑，不是 solver 本体推导。

## 三类文档的边界

- `Vector_Thrust_UAV_Discrete_Model_First_Order_Lag.md` 说明算法模型和代价函数。
- `ilqr_px4_deployment_design.md` 说明 PX4 中的代码部署、运行链路和接口边界。
- `bias_estimator_design.md` 说明部署层低频扰动补偿和 anti-windup 策略。

如果要修改 `ThrustVectorFirstOrderLag.hpp` 中的状态、输入、动力学或代价形式，应先更新算法模型文档，再同步检查 PX4 部署文档。

如果要修改 `ThrustVectorPositionControl.cpp`、`PositionControl.cpp` 中的调度、状态来源、uORB 接口、failsafe、限幅或姿态输出，应优先更新 PX4 部署代码设计文档。

如果要修改 `AccelerationBiasEstimator.hpp`、bias 参数或 `_velocityControl()` 中的 bias 更新/冻结逻辑，应优先更新 bias estimator 设计文档。
