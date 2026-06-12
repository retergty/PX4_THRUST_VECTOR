#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

#include "AugmentedLagrangian/StateInputAugmentedLagrangian.hpp"
#include "Constraint/StateInputConstraint.hpp"
#include "Cost/Cost.hpp"
#include "Dynamics/DiscreteSystemDynamicsBase.hpp"
#include "Initialization/Initializer.hpp"
#include "Misc/LinearInterpolation.hpp"
#include "OptimalControl/OptimalControlProblem.hpp"
#include "Penalties/SlacknessSquaredHingePenalty.hpp"
#include "iLQR/DDPSetting.hpp"
#include "iLQR/LinearAlgebraTypes.hpp"
#include "iLQR/iLQR.hpp"
#include "iLQR/iLQRDescriptor.hpp"
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * 该示例定义了带一阶低通执行器的速度外环离散最优控制问题（有约束版本）。
 *
 * 当前模型不是完整的矢量推力无人机刚体动力学，而是在预测时域内
 * 姿态已知且近似不变时的 NED 平动增量 MPC 模型；为描述内环、电机或
 * 矢量推力机构的慢响应，在增量式总加速度模型上加入一阶低通执行器：
 * - 状态 x = [v, a_eff, a_cmd_prev]，其中 v 为 NED 速度，a_eff 为当前
 *   实际生效的 NED 总加速度（宜来自估计或执行器响应），a_cmd_prev 为上一拍
 *   发送给下游的 NED 命令总加速度（应写入实际发送值，而非限幅前期望指令）。
 * - 输入 u 为本拍 NED 命令总加速度增量 Δa_cmd，不是机体系推力。
 * - 动力学：a_cmd,k = a_cmd_prev + u，
 *           a_eff,k+1 = (1-α)a_eff,k + α a_cmd,k，
 *           v_{k+1} = v_k + dt * a_eff,k+1，
 *           a_cmd_prev_{k+1} = a_cmd,k。
 *   其中 α ∈ [0,1] 为离散一阶低通系数，α=1 表示无滞后。
 *
 * 推力方向变化代价基于实际生效总加速度 a_eff（非命令加速度），先减去重力
 * 得到推力加速度，惩罚相邻两拍推力方向变化；低推力区通过门控权重减弱该代价。
 *
 * 与无约束版本不同，本文件通过增广拉格朗日接入了 state-input 不等式约束：
 * 1) 推力命令加速度锥约束 + 命令总加速度椭球约束（2 维 term）
 * 2) 推力命令加速度 z 轴最小值约束（1 维 term）
 */

namespace tvfol_constrain {
static constexpr int STATE_DIM = 9;
static constexpr int INPUT_DIM = 3;
static constexpr float kGravity = 9.80665f;

template <typename Scalar>
using ThrustVectorConstraintConfig = ConstraintConfig<
    StateConstraintConfig<ConstraintLayout<>>,
    StateInputConstraintConfig<ConstraintLayout<
        ConstraintGroupLayout<>,
        ConstraintGroupLayout<ConstraintTerm<2>, ConstraintTerm<1>>>>,
    FinalStateConstraintConfig<ConstraintLayout<>>>;

// 有约束问题类型：中间阶段包含 2 个 state-input 不等式 term，
// 分别为 [锥+椭球] 与 [z 轴最小值]。
template <typename Scalar, size_t PredictLength>
using ThrustVectorProblem = OptimalControlProblem<
    Scalar,
    TranscriptionConfig<Dimensions<STATE_DIM, INPUT_DIM>,
                        Horizon<PredictLength>, DiscreteDynamics>,
    ThrustVectorConstraintConfig<Scalar>>;

template <typename Scalar>
class ThrustVectorDynamicSystem final
    : public DiscreteSystemDynamicsBase<Scalar, STATE_DIM, INPUT_DIM> {
 public:
  using LinearApproximation_t =
      typename DiscreteSystemDynamicsBase<Scalar, STATE_DIM,
                                          INPUT_DIM>::LinearApproximation_t;

  struct PreCompCache {
    PreCompCache() {
      approximation.setZero();
      dt = 0;
      alpha = 1;
      oneMinusAlpha = 0;
      velocityEffectiveCoefficient = 0;
      velocityCommandCoefficient = 0;
      dirty = true;
    }

    LinearApproximation_t approximation;
    Scalar dt;
    Scalar alpha;
    Scalar oneMinusAlpha;
    Scalar velocityEffectiveCoefficient;
    Scalar velocityCommandCoefficient;
    bool dirty;
  };

  explicit ThrustVectorDynamicSystem(const Scalar alpha = Scalar(1))
      : alpha_(alpha) {
    assert(alpha_ >= Scalar(0));
    assert(alpha_ <= Scalar(1));
  }

  ~ThrustVectorDynamicSystem() override = default;

  void updateCache(const Scalar dt) {
    if (cache_.dirty ||
        std::abs(cache_.dt - dt) > std::numeric_limits<Scalar>::epsilon() ||
        std::abs(cache_.alpha - alpha_) >
            std::numeric_limits<Scalar>::epsilon()) {
      const Scalar alpha = alpha_;
      const Scalar oneMinusAlpha = Scalar(1) - alpha;
      const Scalar velocityEffectiveCoefficient = dt * oneMinusAlpha;
      const Scalar velocityCommandCoefficient = dt * alpha;

      cache_.approximation.setZero();

      for (int i = 0; i < 3; ++i) {
        cache_.approximation.dfdx(i, i) = Scalar(1);
        cache_.approximation.dfdx(i, i + 3) = velocityEffectiveCoefficient;
        cache_.approximation.dfdx(i, i + 6) = velocityCommandCoefficient;

        cache_.approximation.dfdx(i + 3, i + 3) = oneMinusAlpha;
        cache_.approximation.dfdx(i + 3, i + 6) = alpha;

        cache_.approximation.dfdx(i + 6, i + 6) = Scalar(1);

        cache_.approximation.dfdu(i, i) = velocityCommandCoefficient;
        cache_.approximation.dfdu(i + 3, i) = alpha;
        cache_.approximation.dfdu(i + 6, i) = Scalar(1);
      }

      cache_.dt = dt;
      cache_.alpha = alpha;
      cache_.oneMinusAlpha = oneMinusAlpha;
      cache_.velocityEffectiveCoefficient = velocityEffectiveCoefficient;
      cache_.velocityCommandCoefficient = velocityCommandCoefficient;
      cache_.dirty = false;
    }
  }

  void linearApproximation(Scalar t, const Vector<Scalar, STATE_DIM>& x,
                           const Vector<Scalar, INPUT_DIM>& u, Scalar dt,
                           LinearApproximation_t& approximation) override {
    (void)t;

    updateCache(dt);
    approximation = cache_.approximation;
    approximation.f = computeMapCached(x, u);
  }

  void deviationLinearApproximation(
      Scalar t, const Vector<Scalar, STATE_DIM>& x,
      const Vector<Scalar, INPUT_DIM>& u, Scalar dt,
      LinearApproximation_t& approximation) override {
    (void)t;
    (void)x;
    (void)u;

    updateCache(dt);
    approximation = cache_.approximation;
  }

  Vector<Scalar, STATE_DIM> computeMap(Scalar t,
                                       const Vector<Scalar, STATE_DIM>& x,
                                       const Vector<Scalar, INPUT_DIM>& u,
                                       Scalar dt) override {
    (void)t;

    updateCache(dt);
    return computeMapCached(x, u);
  }

 private:
  Vector<Scalar, STATE_DIM> computeMapCached(
      const Vector<Scalar, STATE_DIM>& x,
      const Vector<Scalar, INPUT_DIM>& u) const {
    Vector<Scalar, STATE_DIM> nextState;
    for (int i = 0; i < 3; ++i) {
      const Scalar commandAcceleration = x(i + 6) + u(i);
      nextState(i) = x(i) + cache_.velocityEffectiveCoefficient * x(i + 3) +
                     cache_.velocityCommandCoefficient * commandAcceleration;
      nextState(i + 3) =
          cache_.oneMinusAlpha * x(i + 3) + cache_.alpha * commandAcceleration;
      nextState(i + 6) = commandAcceleration;
    }
    return nextState;
  }

  Scalar alpha_;
  PreCompCache cache_;
};

// 轨迹跟踪代价：跟踪参考速度，用 R 惩罚命令加速度增量，并用 Ra
// 惩罚下一拍命令总加速度 a_cmd_prev + delta_a_cmd 相对参考命令加速度的偏差。
template <typename Scalar, int ArrayLength>
class ThrustVectorTrackCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorTrackCost(const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q,
                        const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& R,
                        const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& Ra,
                        int cost_number)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        Qv_(Q.template topLeftCorner<3, 3>()),
        R_(R),
        Ra_(Ra) {}

  ~ThrustVectorTrackCost() override = default;

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(indexAlpha, stateTrajectory);
    const Vector<Scalar, INPUT_DIM> inputDeviation =
        input - interpolateInputReference(indexAlpha, inputTrajectory);
    const Vector<Scalar, INPUT_DIM> commandAccelerationDeviation =
        state.template segment<3>(6) + input -
        interpolateCommandAccelerationReference(indexAlpha, stateTrajectory);
    const Vector<Scalar, 3> weightedVelocityDeviation = Qv_ * velocityDeviation;
    const Vector<Scalar, INPUT_DIM> weightedInputDeviation =
        R_ * inputDeviation;
    const Vector<Scalar, INPUT_DIM> weightedCommandAccelerationDeviation =
        Ra_ * commandAccelerationDeviation;

    return Scalar(0.5) * velocityDeviation.dot(weightedVelocityDeviation) +
           Scalar(0.5) * inputDeviation.dot(weightedInputDeviation) +
           Scalar(0.5) * commandAccelerationDeviation.dot(
                             weightedCommandAccelerationDeviation);
  }

  void addQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory,
      ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>&
          addAppro) override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(indexAlpha, stateTrajectory);
    const Vector<Scalar, INPUT_DIM> inputDeviation =
        input - interpolateInputReference(indexAlpha, inputTrajectory);
    const Vector<Scalar, INPUT_DIM> commandAccelerationDeviation =
        state.template segment<3>(6) + input -
        interpolateCommandAccelerationReference(indexAlpha, stateTrajectory);
    const Vector<Scalar, 3> weightedVelocityDeviation = Qv_ * velocityDeviation;
    const Vector<Scalar, INPUT_DIM> weightedInputDeviation =
        R_ * inputDeviation;
    const Vector<Scalar, INPUT_DIM> weightedCommandAccelerationDeviation =
        Ra_ * commandAccelerationDeviation;

    addAppro.f +=
        Scalar(0.5) * velocityDeviation.dot(weightedVelocityDeviation) +
        Scalar(0.5) * inputDeviation.dot(weightedInputDeviation) +
        Scalar(0.5) * commandAccelerationDeviation.dot(
                          weightedCommandAccelerationDeviation);
    addAppro.dfdx.template head<3>() += weightedVelocityDeviation;
    addAppro.dfdx.template segment<3>(6) +=
        weightedCommandAccelerationDeviation;
    addAppro.dfdu +=
        weightedInputDeviation + weightedCommandAccelerationDeviation;
    addAppro.dfdxx.template topLeftCorner<3, 3>() += Qv_;
    addAppro.dfdxx.template slice<3, 3>(6, 6) += Ra_;
    addAppro.dfduu += R_ + Ra_;
    addAppro.dfdux.template slice<3, 3>(0, 6) += Ra_;
  }

 private:
  ThrustVectorTrackCost(const ThrustVectorTrackCost& other) = default;

  Vector<Scalar, 3> interpolateVelocityReference(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory)
      const {
    return LinearInterpolation::interpolate(
        indexAlpha, stateTrajectory,
        [](const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& trajectory,
           size_t index) -> Vector<Scalar, 3> {
          return trajectory[index].template head<3>();
        });
  }

  Vector<Scalar, INPUT_DIM> interpolateInputReference(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      const {
    return LinearInterpolation::interpolate(indexAlpha, inputTrajectory);
  }

  Vector<Scalar, INPUT_DIM> interpolateCommandAccelerationReference(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory)
      const {
    return LinearInterpolation::interpolate(
        indexAlpha, stateTrajectory,
        [](const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& trajectory,
           size_t index) -> Vector<Scalar, INPUT_DIM> {
          return trajectory[index].template segment<3>(6);
        });
  }

  Matrix<Scalar, 3, 3> Qv_;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> R_;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> Ra_;
};

// 轨迹跟踪代价的对角版本：只使用 Q、R、Ra
// 对角线，降低小维度问题中的矩阵运算开销。
template <typename Scalar, int ArrayLength>
class ThrustVectorDiagonalTrackCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorDiagonalTrackCost(const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q,
                                const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& R,
                                const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& Ra,
                                int cost_number)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number) {
    for (int i = 0; i < 3; ++i) {
      QvDiagonal_(i) = Q(i, i);
      RDiagonal_(i) = R(i, i);
      RaDiagonal_(i) = Ra(i, i);
    }
  }

  ~ThrustVectorDiagonalTrackCost() override = default;

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    Scalar value = 0;
    for (int i = 0; i < 3; ++i) {
      const Scalar velocityDeviation =
          state(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory, i);
      const Scalar inputDeviation =
          input(i) -
          interpolateInputReferenceComponent(indexAlpha, inputTrajectory, i);
      const Scalar commandAccelerationDeviation =
          state(i + 6) + input(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory,
                                             i + 6);
      const Scalar weightedCommandAccelerationDeviation =
          RaDiagonal_(i) * commandAccelerationDeviation;

      value +=
          QvDiagonal_(i) * velocityDeviation * velocityDeviation +
          RDiagonal_(i) * inputDeviation * inputDeviation +
          commandAccelerationDeviation * weightedCommandAccelerationDeviation;
    }
    return Scalar(0.5) * value;
  }

  void addQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory,
      ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>&
          addAppro) override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    Scalar value = 0;
    for (int i = 0; i < 3; ++i) {
      const Scalar velocityDeviation =
          state(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory, i);
      const Scalar inputDeviation =
          input(i) -
          interpolateInputReferenceComponent(indexAlpha, inputTrajectory, i);
      const Scalar commandAccelerationDeviation =
          state(i + 6) + input(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory,
                                             i + 6);
      const Scalar weightedCommandAccelerationDeviation =
          RaDiagonal_(i) * commandAccelerationDeviation;

      value +=
          QvDiagonal_(i) * velocityDeviation * velocityDeviation +
          RDiagonal_(i) * inputDeviation * inputDeviation +
          commandAccelerationDeviation * weightedCommandAccelerationDeviation;

      addAppro.dfdx(i) += QvDiagonal_(i) * velocityDeviation;
      addAppro.dfdx(i + 6) += weightedCommandAccelerationDeviation;
      addAppro.dfdu(i) +=
          RDiagonal_(i) * inputDeviation + weightedCommandAccelerationDeviation;
      addAppro.dfdxx(i, i) += QvDiagonal_(i);
      addAppro.dfdxx(i + 6, i + 6) += RaDiagonal_(i);
      addAppro.dfduu(i, i) += RDiagonal_(i) + RaDiagonal_(i);
      addAppro.dfdux(i, i + 6) += RaDiagonal_(i);
    }
    addAppro.f += Scalar(0.5) * value;
  }

 private:
  ThrustVectorDiagonalTrackCost(const ThrustVectorDiagonalTrackCost& other) =
      default;

  Scalar interpolateStateReferenceComponent(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      int dim) const {
    if constexpr (ArrayLength > 1) {
      const int index = indexAlpha.first;
      const Scalar alpha = indexAlpha.second;
      return alpha * stateTrajectory[index](dim) +
             (Scalar(1) - alpha) * stateTrajectory[index + 1](dim);
    } else {
      return stateTrajectory[0](dim);
    }
  }

  Scalar interpolateInputReferenceComponent(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory,
      int dim) const {
    if constexpr (ArrayLength > 1) {
      const int index = indexAlpha.first;
      const Scalar alpha = indexAlpha.second;
      return alpha * inputTrajectory[index](dim) +
             (Scalar(1) - alpha) * inputTrajectory[index + 1](dim);
    } else {
      return inputTrajectory[0](dim);
    }
  }

  Vector<Scalar, 3> QvDiagonal_;
  Vector<Scalar, INPUT_DIM> RDiagonal_;
  Vector<Scalar, INPUT_DIM> RaDiagonal_;
};

// 终端速度跟踪代价：在预测时域末端惩罚速度与参考速度的偏差。
template <typename Scalar, int ArrayLength>
class ThrustVectorTrackFinalCost final
    : public StateCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorTrackFinalCost(const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q,
                             int cost_number)
      : StateCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        Qv_(Q.template topLeftCorner<3, 3>()) {}

  ~ThrustVectorTrackFinalCost() override = default;

  Scalar getValue(Scalar time, const Vector<Scalar, STATE_DIM>& state,
                  const std::array<Scalar, ArrayLength>& timeTrajectory,
                  const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>&
                      stateTrajectory) override {
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(time, timeTrajectory, stateTrajectory);
    return Scalar(0.5) * velocityDeviation.dot(Qv_ * velocityDeviation);
  }

  void addQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>&
          addAppro) override {
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(time, timeTrajectory, stateTrajectory);
    const Vector<Scalar, 3> weightedVelocityDeviation = Qv_ * velocityDeviation;

    addAppro.f +=
        Scalar(0.5) * velocityDeviation.dot(weightedVelocityDeviation);
    addAppro.dfdx.template head<3>() += weightedVelocityDeviation;
    addAppro.dfdxx.template topLeftCorner<3, 3>() += Qv_;
  }

 private:
  ThrustVectorTrackFinalCost(const ThrustVectorTrackFinalCost& other) = default;

  Vector<Scalar, 3> interpolateVelocityReference(
      Scalar time, const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory)
      const {
    return LinearInterpolation::interpolate(
        time, timeTrajectory, stateTrajectory,
        [](const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& trajectory,
           size_t index) -> Vector<Scalar, 3> {
          return trajectory[index].template head<3>();
        });
  }

  Matrix<Scalar, 3, 3> Qv_;
};

// 终端速度跟踪代价的对角版本：只使用 Qf 对角线按轴惩罚末端速度误差。
template <typename Scalar, int ArrayLength>
class ThrustVectorDiagonalTrackFinalCost final
    : public StateCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorDiagonalTrackFinalCost(
      const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q, int cost_number)
      : StateCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number) {
    for (int i = 0; i < 3; ++i) {
      QvDiagonal_(i) = Q(i, i);
    }
  }

  ~ThrustVectorDiagonalTrackFinalCost() override = default;

  Scalar getValue(Scalar time, const Vector<Scalar, STATE_DIM>& state,
                  const std::array<Scalar, ArrayLength>& timeTrajectory,
                  const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>&
                      stateTrajectory) override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    Scalar value = 0;
    for (int i = 0; i < 3; ++i) {
      const Scalar velocityDeviation =
          state(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory, i);
      value += QvDiagonal_(i) * velocityDeviation * velocityDeviation;
    }
    return Scalar(0.5) * value;
  }

  void addQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>&
          addAppro) override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    Scalar value = 0;
    for (int i = 0; i < 3; ++i) {
      const Scalar velocityDeviation =
          state(i) -
          interpolateStateReferenceComponent(indexAlpha, stateTrajectory, i);
      value += QvDiagonal_(i) * velocityDeviation * velocityDeviation;

      addAppro.dfdx(i) += QvDiagonal_(i) * velocityDeviation;
      addAppro.dfdxx(i, i) += QvDiagonal_(i);
    }
    addAppro.f += Scalar(0.5) * value;
  }

 private:
  ThrustVectorDiagonalTrackFinalCost(
      const ThrustVectorDiagonalTrackFinalCost& other) = default;

  Scalar interpolateStateReferenceComponent(
      const std::pair<int, Scalar>& indexAlpha,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      int dim) const {
    if constexpr (ArrayLength > 1) {
      const int index = indexAlpha.first;
      const Scalar alpha = indexAlpha.second;
      return alpha * stateTrajectory[index](dim) +
             (Scalar(1) - alpha) * stateTrajectory[index + 1](dim);
    } else {
      return stateTrajectory[0](dim);
    }
  }

  Vector<Scalar, 3> QvDiagonal_;
};

// 推力方向变化代价：基于一阶滞后后的实际生效加速度，惩罚相邻时刻推力方向突变。
template <typename Scalar, int ArrayLength>
class ThrustDirectionChangeCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  static constexpr Scalar epsilon = Scalar(1e-4);
  static constexpr Scalar MinThrustAccelerationForDirection = Scalar(1e-2);

  ThrustDirectionChangeCost(const Scalar alpha, const Scalar weight,
                            int cost_number)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        alpha_(alpha),
        oneMinusAlpha_(Scalar(1) - alpha),
        weight_(weight),
        gravityVector_{Scalar(0), Scalar(0), Scalar(kGravity)} {
    assert(alpha_ >= Scalar(0));
    assert(alpha_ <= Scalar(1));
  }

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    (void)time;
    (void)timeTrajectory;
    (void)inputTrajectory;
    (void)stateTrajectory;

    const Vector<Scalar, 3> lastThrustAcceleration =
        effectiveAcceleration(state) - gravityVector_;
    const Vector<Scalar, 3> thrustAcceleration =
        nextEffectiveAcceleration(state, input) - gravityVector_;
    const Vector<Scalar, 3> lastThrustDirection =
        lastThrustAcceleration /
        std::sqrt(lastThrustAcceleration.dot(lastThrustAcceleration) + epsilon);
    const Vector<Scalar, 3> thrustDirection =
        thrustAcceleration /
        std::sqrt(thrustAcceleration.dot(thrustAcceleration) + epsilon);
    const Vector<Scalar, 3> rk = thrustDirection - lastThrustDirection;
    const Scalar gate =
        lowAccelerationGate(lastThrustAcceleration, thrustAcceleration);
    return Scalar(0.5) * gate * weight_ * rk.dot(rk);
  }

  void addQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory,
      ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>&
          addAppro) override {
    (void)time;
    (void)timeTrajectory;
    (void)inputTrajectory;
    (void)stateTrajectory;

    const Vector<Scalar, 3> lastThrustAcceleration =
        effectiveAcceleration(state) - gravityVector_;
    const Vector<Scalar, 3> thrustAcceleration =
        nextEffectiveAcceleration(state, input) - gravityVector_;
    const Scalar lastThrustAccelerationNorm =
        std::sqrt(lastThrustAcceleration.dot(lastThrustAcceleration) + epsilon);
    const Scalar thrustAccelerationNorm =
        std::sqrt(thrustAcceleration.dot(thrustAcceleration) + epsilon);
    const Vector<Scalar, 3> lastThrustDirection =
        lastThrustAcceleration / lastThrustAccelerationNorm;
    const Vector<Scalar, 3> thrustDirection =
        thrustAcceleration / thrustAccelerationNorm;
    const Vector<Scalar, 3> rk = thrustDirection - lastThrustDirection;

    const Matrix<Scalar, 3, 3> lastThrustAccelerationJacobian =
        computeDirectionJacobian(lastThrustAcceleration,
                                 lastThrustAccelerationNorm);
    const Matrix<Scalar, 3, 3> thrustAccelerationJacobian =
        computeDirectionJacobian(thrustAcceleration, thrustAccelerationNorm);
    const Matrix<Scalar, 3, 3> effectiveAccelerationJacobian =
        oneMinusAlpha_ * thrustAccelerationJacobian -
        lastThrustAccelerationJacobian;
    const Matrix<Scalar, 3, 3> commandAccelerationJacobian =
        alpha_ * thrustAccelerationJacobian;

    const Scalar gate =
        lowAccelerationGate(lastThrustAcceleration, thrustAcceleration);
    const Scalar effectiveWeight = gate * weight_;
    // 命令状态 x(6:8) 与输入 u 经过相同的链路雅可比（均为 alpha * J），
    // 因此其一阶项可共享同一中间量。
    const Vector<Scalar, 3> effectiveGradient =
        effectiveWeight * effectiveAccelerationJacobian.transpose() * rk;
    const Vector<Scalar, 3> commandGradient =
        effectiveWeight * commandAccelerationJacobian.transpose() * rk;
    // 预先构造对称块，复用到 dfdxx/dfduu/dfdux，避免重复 3x3 矩阵乘法。
    const Matrix<Scalar, 3, 3> effectiveHessian =
        effectiveWeight * effectiveAccelerationJacobian.transpose() *
        effectiveAccelerationJacobian;
    const Matrix<Scalar, 3, 3> effectiveCommandHessian =
        effectiveWeight * effectiveAccelerationJacobian.transpose() *
        commandAccelerationJacobian;
    const Matrix<Scalar, 3, 3> commandHessian =
        effectiveWeight * commandAccelerationJacobian.transpose() *
        commandAccelerationJacobian;

    addAppro.f += Scalar(0.5) * effectiveWeight * rk.dot(rk);
    addAppro.dfdx.template segment<3>(3) += effectiveGradient;
    addAppro.dfdx.template segment<3>(6) += commandGradient;
    addAppro.dfdu += commandGradient;

    addAppro.dfdxx.template slice<3, 3>(3, 3) += effectiveHessian;
    addAppro.dfdxx.template slice<3, 3>(3, 6) += effectiveCommandHessian;
    addAppro.dfdxx.template slice<3, 3>(6, 3) +=
        effectiveCommandHessian.transpose();
    addAppro.dfdxx.template slice<3, 3>(6, 6) += commandHessian;
    addAppro.dfduu += commandHessian;
    addAppro.dfdux.template slice<3, 3>(0, 3) +=
        effectiveCommandHessian.transpose();
    addAppro.dfdux.template slice<3, 3>(0, 6) += commandHessian;
  }

 private:
  static Matrix<Scalar, 3, 3> computeDirectionJacobian(
      const Vector<Scalar, 3>& acceleration, const Scalar norm) {
    Matrix<Scalar, 3, 3> jacobian;
    const Scalar invNorm = Scalar(1) / norm;
    const Scalar invNormCubed = invNorm * invNorm * invNorm;

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        jacobian(i, j) = (i == j ? invNorm : Scalar(0)) -
                         acceleration(i) * acceleration(j) * invNormCubed;
      }
    }
    return jacobian;
  }

  static Scalar accelerationGateSigma(const Vector<Scalar, 3>& acceleration) {
    const Scalar accelerationNormSquared = acceleration.squaredNorm();
    const Scalar minAccelerationSquared =
        MinThrustAccelerationForDirection * MinThrustAccelerationForDirection;
    return accelerationNormSquared /
           (accelerationNormSquared + minAccelerationSquared);
  }

  static Scalar lowAccelerationGate(
      const Vector<Scalar, 3>& lastAcceleration,
      const Vector<Scalar, 3>& currentAcceleration) {
    return accelerationGateSigma(lastAcceleration) *
           accelerationGateSigma(currentAcceleration);
  }

  static Vector<Scalar, 3> effectiveAcceleration(
      const Vector<Scalar, STATE_DIM>& state) {
    Vector<Scalar, 3> acceleration;
    for (int i = 0; i < 3; ++i) {
      acceleration(i) = state(i + 3);
    }
    return acceleration;
  }

  Vector<Scalar, 3> nextEffectiveAcceleration(
      const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const {
    Vector<Scalar, 3> acceleration;
    for (int i = 0; i < 3; ++i) {
      const Scalar commandAcceleration = state(i + 6) + input(i);
      acceleration(i) =
          oneMinusAlpha_ * state(i + 3) + alpha_ * commandAcceleration;
    }
    return acceleration;
  }

  Scalar alpha_;
  Scalar oneMinusAlpha_;
  const Scalar weight_;
  const Vector<Scalar, 3> gravityVector_;
};

// 命令加速度约束：锥约束使用推力命令加速度 a_T,cmd = a_cmd - g * e3，
// 椭球约束使用命令总加速度 a_cmd = x(6:8) + u。
template <typename Scalar>
class ThrustCommandAccelerationConstraint final
    : public StateInputConstraint<Scalar, STATE_DIM, INPUT_DIM, 2> {
 public:
  struct Config {
    Scalar maxTiltAngleRad{Scalar(0.7853981633974483)};  // 默认 45 deg。
    Scalar axMax{Scalar(6.0)};           // 总加速度椭球 x 轴半径。
    Scalar ayMax{Scalar(6.0)};           // 总加速度椭球 y 轴半径。
    Scalar azMax{Scalar(6.0)};           // 总加速度椭球 z 轴半径。
    Scalar coneSmoothing{Scalar(1e-6)};  // 锥约束平滑项 epsilon。
  };

  explicit ThrustCommandAccelerationConstraint(const Config& config)
      : StateInputConstraint<Scalar, STATE_DIM, INPUT_DIM, 2>(
            ConstraintOrder::Quadratic),
        tanMaxTiltAngle_(std::tan(config.maxTiltAngleRad)),
        inverseAxMaxSquared_(Scalar(1) / (config.axMax * config.axMax)),
        inverseAyMaxSquared_(Scalar(1) / (config.ayMax * config.ayMax)),
        inverseAzMaxSquared_(Scalar(1) / (config.azMax * config.azMax)),
        coneSmoothing_(config.coneSmoothing),
        gravity_(Scalar(kGravity)) {
    assert(config.maxTiltAngleRad > Scalar(0));
    assert(config.maxTiltAngleRad < Scalar(1.5707963267948966));
    assert(config.axMax > Scalar(0));
    assert(config.ayMax > Scalar(0));
    assert(config.azMax > Scalar(0));
    assert(config.coneSmoothing > Scalar(0));
  }

  ~ThrustCommandAccelerationConstraint() override = default;

  Vector<Scalar, 2> getValue(
      const Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const override {
    (void)time;

    const CommandAcceleration aCmd = commandAcceleration(state, input);
    const CommandThrustAcceleration aT = commandThrustAcceleration(aCmd);
    const Scalar radialNorm =
        std::sqrt(aT.ax * aT.ax + aT.ay * aT.ay + coneSmoothing_);

    Vector<Scalar, 2> value;
    // h_cone >= 0: sqrt(t_x^2 + t_y^2) <= -tan(theta) * t_z
    value(0) = -tanMaxTiltAngle_ * aT.az - radialNorm;
    // h_ellip >= 0: total command acceleration stays inside the ellipsoid.
    value(1) = Scalar(1) - inverseAxMaxSquared_ * aCmd.ax * aCmd.ax -
               inverseAyMaxSquared_ * aCmd.ay * aCmd.ay -
               inverseAzMaxSquared_ * aCmd.az * aCmd.az;
    return value;
  }

  VectorFunctionQuadraticApproximation<Scalar, 2, STATE_DIM, INPUT_DIM>
  getQuadraticApproximation(
      const Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const override {
    (void)time;

    VectorFunctionQuadraticApproximation<Scalar, 2, STATE_DIM, INPUT_DIM>
        approximation;
    approximation.setZero();
    approximation.f = getValue(time, state, input);

    const CommandAcceleration aCmd = commandAcceleration(state, input);
    const CommandThrustAcceleration aT = commandThrustAcceleration(aCmd);

    // ----- 约束 0：锥约束 h_cone -----
    const Scalar radialNormSquared =
        aT.ax * aT.ax + aT.ay * aT.ay + coneSmoothing_;
    const Scalar radialNorm = std::sqrt(radialNormSquared);
    const Scalar inverseRadialNorm = Scalar(1) / radialNorm;
    const Scalar inverseRadialNormCubed =
        inverseRadialNorm * inverseRadialNorm * inverseRadialNorm;

    const Scalar coneDax = -aT.ax * inverseRadialNorm;
    const Scalar coneDay = -aT.ay * inverseRadialNorm;
    const Scalar coneDaz = -tanMaxTiltAngle_;

    const Scalar coneDaxDax =
        -inverseRadialNorm + aT.ax * aT.ax * inverseRadialNormCubed;
    const Scalar coneDayDay =
        -inverseRadialNorm + aT.ay * aT.ay * inverseRadialNormCubed;
    const Scalar coneDaxDay = aT.ax * aT.ay * inverseRadialNormCubed;

    approximation.dfdx(0, 6) = coneDax;
    approximation.dfdx(0, 7) = coneDay;
    approximation.dfdx(0, 8) = coneDaz;
    approximation.dfdu(0, 0) = coneDax;
    approximation.dfdu(0, 1) = coneDay;
    approximation.dfdu(0, 2) = coneDaz;

    approximation.dfdxx[0](6, 6) = coneDaxDax;
    approximation.dfdxx[0](7, 7) = coneDayDay;
    approximation.dfdxx[0](6, 7) = coneDaxDay;
    approximation.dfdxx[0](7, 6) = coneDaxDay;

    approximation.dfduu[0](0, 0) = coneDaxDax;
    approximation.dfduu[0](1, 1) = coneDayDay;
    approximation.dfduu[0](0, 1) = coneDaxDay;
    approximation.dfduu[0](1, 0) = coneDaxDay;

    approximation.dfdux[0](0, 6) = coneDaxDax;
    approximation.dfdux[0](1, 7) = coneDayDay;
    approximation.dfdux[0](0, 7) = coneDaxDay;
    approximation.dfdux[0](1, 6) = coneDaxDay;

    // ----- 约束 1：椭球约束 h_ellip（总加速度） -----
    const Scalar ellipDax = -Scalar(2) * inverseAxMaxSquared_ * aCmd.ax;
    const Scalar ellipDay = -Scalar(2) * inverseAyMaxSquared_ * aCmd.ay;
    const Scalar ellipDaz = -Scalar(2) * inverseAzMaxSquared_ * aCmd.az;

    const Scalar ellipDaxDax = -Scalar(2) * inverseAxMaxSquared_;
    const Scalar ellipDayDay = -Scalar(2) * inverseAyMaxSquared_;
    const Scalar ellipDazDaz = -Scalar(2) * inverseAzMaxSquared_;

    approximation.dfdx(1, 6) = ellipDax;
    approximation.dfdx(1, 7) = ellipDay;
    approximation.dfdx(1, 8) = ellipDaz;
    approximation.dfdu(1, 0) = ellipDax;
    approximation.dfdu(1, 1) = ellipDay;
    approximation.dfdu(1, 2) = ellipDaz;

    approximation.dfdxx[1](6, 6) = ellipDaxDax;
    approximation.dfdxx[1](7, 7) = ellipDayDay;
    approximation.dfdxx[1](8, 8) = ellipDazDaz;

    approximation.dfduu[1](0, 0) = ellipDaxDax;
    approximation.dfduu[1](1, 1) = ellipDayDay;
    approximation.dfduu[1](2, 2) = ellipDazDaz;

    approximation.dfdux[1](0, 6) = ellipDaxDax;
    approximation.dfdux[1](1, 7) = ellipDayDay;
    approximation.dfdux[1](2, 8) = ellipDazDaz;

    return approximation;
  }

 private:
  struct CommandAcceleration {
    Scalar ax;
    Scalar ay;
    Scalar az;
  };

  struct CommandThrustAcceleration {
    Scalar ax;
    Scalar ay;
    Scalar az;
  };

  static CommandAcceleration commandAcceleration(
      const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) {
    return {state(6) + input(0), state(7) + input(1), state(8) + input(2)};
  }

  CommandThrustAcceleration commandThrustAcceleration(
      const CommandAcceleration& commandAcceleration) const {
    return {commandAcceleration.ax, commandAcceleration.ay,
            commandAcceleration.az - gravity_};
  }

  Scalar tanMaxTiltAngle_;
  Scalar inverseAxMaxSquared_;
  Scalar inverseAyMaxSquared_;
  Scalar inverseAzMaxSquared_;
  Scalar coneSmoothing_;
  Scalar gravity_;
};

// 推力命令加速度线性约束：z 轴最小值约束（防反推）。
// 约束对象为 a_T,cmd = a_cmd - g * e3，其中 a_cmd = x(6:8) + u。
template <typename Scalar>
class ThrustCommandAccelerationZMinConstraint final
    : public StateInputConstraint<Scalar, STATE_DIM, INPUT_DIM, 1> {
 public:
  struct Config {
    Scalar zMin{Scalar(-0.01)};  // h_z >= 0: zMin - a_T,cmd,z
  };

  explicit ThrustCommandAccelerationZMinConstraint(const Config& config)
      : StateInputConstraint<Scalar, STATE_DIM, INPUT_DIM, 1>(
            ConstraintOrder::Linear),
        zMin_(config.zMin),
        gravity_(Scalar(kGravity)) {}

  ~ThrustCommandAccelerationZMinConstraint() override = default;

  Vector<Scalar, 1> getValue(
      const Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const override {
    (void)time;
    Vector<Scalar, 1> value;
    value(0) = zMin_ - commandThrustAccelerationZ(state, input);
    return value;
  }

  VectorFunctionLinearApproximation<Scalar, 1, STATE_DIM, INPUT_DIM>
  getLinearApproximation(
      const Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const override {
    VectorFunctionLinearApproximation<Scalar, 1, STATE_DIM, INPUT_DIM>
        approximation;
    approximation.setZero();
    approximation.f = getValue(time, state, input);
    approximation.dfdx(0, 8) = Scalar(-1);
    approximation.dfdu(0, 2) = Scalar(-1);
    return approximation;
  }

 private:
  Scalar commandThrustAccelerationZ(
      const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input) const {
    return state(8) + input(2) - gravity_;
  }

  Scalar zMin_;
  Scalar gravity_;
};

template <typename Scalar>
struct ThrustCommandAccelerationConstraintSettings {
  typename ThrustCommandAccelerationConstraint<Scalar>::Config constraint;
  typename ThrustCommandAccelerationZMinConstraint<Scalar>::Config
      zMinConstraint;
  typename SlacknessSquaredHingePenalty<Scalar>::Config constraintPenalty{
      Scalar(10.0), Scalar(1.0)};
  typename SlacknessSquaredHingePenalty<Scalar>::Config zMinPenalty{
      Scalar(10.0), Scalar(1.0)};
};

// 命令加速度不等式约束的增广拉格朗日封装：
// - term 0：推力锥约束 + 总加速度椭球约束（2 维）
// - term 1：z 轴最小值约束（1 维）
template <typename Scalar>
class ThrustCommandAccelerationAugmentedLagrangian final {
 public:
  using Constraint_t = ThrustCommandAccelerationConstraint<Scalar>;
  using ZMinConstraint_t = ThrustCommandAccelerationZMinConstraint<Scalar>;
  using Penalty_t = SlacknessSquaredHingePenalty<Scalar>;
  using ConstraintAugmentedLagrangian_t =
      StateInputAugmentedLagrangian<Scalar, STATE_DIM, INPUT_DIM, 2>;
  using ZMinAugmentedLagrangian_t =
      StateInputAugmentedLagrangian<Scalar, STATE_DIM, INPUT_DIM, 1>;
  using Settings_t = ThrustCommandAccelerationConstraintSettings<Scalar>;

  explicit ThrustCommandAccelerationAugmentedLagrangian(
      const Settings_t& config)
      : constraint_(config.constraint),
        zMinConstraint_(config.zMinConstraint),
        constraintPenalty_(config.constraintPenalty),
        zMinPenalty_(config.zMinPenalty),
        constraintLagrangian_(&constraint_, &constraintPenalty_),
        zMinLagrangian_(&zMinConstraint_, &zMinPenalty_) {}

  ConstraintAugmentedLagrangian_t* constraintLagrangian() {
    return &constraintLagrangian_;
  }
  const ConstraintAugmentedLagrangian_t* constraintLagrangian() const {
    return &constraintLagrangian_;
  }

  ZMinAugmentedLagrangian_t* zMinLagrangian() { return &zMinLagrangian_; }
  const ZMinAugmentedLagrangian_t* zMinLagrangian() const {
    return &zMinLagrangian_;
  }

 private:
  Constraint_t constraint_;
  ZMinConstraint_t zMinConstraint_;
  Penalty_t constraintPenalty_;
  Penalty_t zMinPenalty_;
  ConstraintAugmentedLagrangian_t constraintLagrangian_;
  ZMinAugmentedLagrangian_t zMinLagrangian_;
};

template <typename Scalar>
class ThrustVectorReferenceTrajectoryGenerator {
 public:
  ThrustVectorReferenceTrajectoryGenerator() = default;

  bool initialized() const { return initialized_; }

  void reset(const Vector<Scalar, 3>& velocity) {
    velocity_reference_ = velocity;
    initialized_ = true;
  }

  void update(const Vector<Scalar, 3>& velocity_setpoint,
              const Vector<Scalar, 3>& alpha) {
    velocity_reference_ =
        lowPass(velocity_reference_, velocity_setpoint, constrainAlpha(alpha));
  }

  Vector<Scalar, 3> currentVelocity() const { return velocity_reference_; }

  template <typename TargetTrajectory>
  void generate(const Scalar current_time, const Scalar time_step,
                const Vector<Scalar, 3>& velocity_setpoint,
                const Vector<Scalar, 3>& command_acceleration_reference,
                TargetTrajectory& targetTrajectory,
                const Vector<Scalar, 3>& alpha) const {
    Vector<Scalar, 3> preview_velocity = velocity_reference_;
    const Vector<Scalar, 3> constrained_alpha = constrainAlpha(alpha);
    Scalar trajectory_time = current_time;

    for (size_t i = 0; i < targetTrajectory.timeTrajectory.size(); ++i) {
      if (i > 0) {
        preview_velocity =
            lowPass(preview_velocity, velocity_setpoint, constrained_alpha);
      }

      targetTrajectory.timeTrajectory[i] = trajectory_time;
      trajectory_time += time_step;
      targetTrajectory.stateTrajectory[i].setZero();
      for (int dim = 0; dim < 3; ++dim) {
        targetTrajectory.stateTrajectory[i](dim) = preview_velocity(dim);
        targetTrajectory.stateTrajectory[i](dim + 6) =
            command_acceleration_reference(dim);
      }

      targetTrajectory.inputTrajectory[i].setZero();
    }
  }

 private:
  static Vector<Scalar, 3> lowPass(const Vector<Scalar, 3>& previous,
                                   const Vector<Scalar, 3>& target,
                                   const Vector<Scalar, 3>& alpha) {
    Vector<Scalar, 3> filtered;

    for (int i = 0; i < 3; ++i) {
      filtered(i) = previous(i) + alpha(i) * (target(i) - previous(i));
    }

    return filtered;
  }

  static Vector<Scalar, 3> constrainAlpha(const Vector<Scalar, 3>& alpha) {
    Vector<Scalar, 3> constrained_alpha;

    for (int i = 0; i < 3; ++i) {
      constrained_alpha(i) = constrainAlphaComponent(alpha(i));
    }

    return constrained_alpha;
  }

  static Scalar constrainAlphaComponent(const Scalar alpha) {
    if (!std::isfinite(alpha)) {
      return Scalar(1);
    }

    if (alpha < Scalar(0)) {
      return Scalar(0);
    }

    if (alpha > Scalar(1)) {
      return Scalar(1);
    }

    return alpha;
  }

  Vector<Scalar, 3> velocity_reference_{Scalar(0), Scalar(0), Scalar(0)};
  bool initialized_{false};
};

template <typename Scalar>
struct ThrustVectorOCPSettings {
  using ConstraintSettings_t =
      ThrustCommandAccelerationConstraintSettings<Scalar>;

  // 权重
  Matrix<Scalar, STATE_DIM, STATE_DIM> Q;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> R;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> Ra;
  Matrix<Scalar, STATE_DIM, STATE_DIM> Qf;
  Scalar weight;

  // 离散一阶低通系数
  Scalar alpha;

  // 速度参考轨迹一阶低通系数
  Vector<Scalar, 3> referenceTrajectoryAlpha{Scalar(1), Scalar(1), Scalar(1)};

  // 推力锥约束、总加速度椭球约束、z-min 约束及其增广惩罚参数。
  ConstraintSettings_t constraintSettings{};
};

template <typename Scalar>
struct ThrustVectorILQRSettings {
  DDPSettings<Scalar> ddpSettings;

  ThrustVectorOCPSettings<Scalar> ocpSettings;
};

template <typename Scalar>
class HoverInitializer final
    : public Initializer<Scalar, STATE_DIM, INPUT_DIM> {
 public:
  explicit HoverInitializer(const Scalar alpha = Scalar(1))
      : alpha_(alpha), oneMinusAlpha_(Scalar(1) - alpha) {
    assert(alpha_ >= Scalar(0));
    assert(alpha_ <= Scalar(1));
  }

  void compute(const Scalar time, const Vector<Scalar, STATE_DIM>& state,
               const Scalar nextTime, Vector<Scalar, INPUT_DIM>& input,
               Vector<Scalar, STATE_DIM>& nextState) override {
    (void)time;
    const Scalar dt = nextTime - time;
    input = hoverInput();

    for (int i = 0; i < 3; ++i) {
      const Scalar commandAcceleration = state(i + 6) + input(i);
      const Scalar effectiveAcceleration =
          oneMinusAlpha_ * state(i + 3) + alpha_ * commandAcceleration;
      nextState(i) = state(i) + dt * effectiveAcceleration;
      nextState(i + 3) = effectiveAcceleration;
      nextState(i + 6) = commandAcceleration;
    }
  }

  static Vector<Scalar, INPUT_DIM> hoverInput() {
    return Vector<Scalar, INPUT_DIM>{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
  }

 private:
  Scalar alpha_;
  Scalar oneMinusAlpha_;
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorOptimalControlProblem {
 public:
  using Problem_t = ThrustVectorProblem<Scalar, PredictLength>;
  using TrackCost_t =
      ThrustVectorDiagonalTrackCost<Scalar,
                                    static_cast<int>(PredictLength + 1)>;
  using DirectionCost_t =
      ThrustDirectionChangeCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using FinalCost_t =
      ThrustVectorDiagonalTrackFinalCost<Scalar,
                                         static_cast<int>(PredictLength + 1)>;
  using ThrustVectorDynamicSystem_t = ThrustVectorDynamicSystem<Scalar>;
  using ConstraintAugmentedLagrangian_t =
      ThrustCommandAccelerationAugmentedLagrangian<Scalar>;

  ThrustVectorOptimalControlProblem(
      const ThrustVectorOCPSettings<Scalar>& settings)
      : trackCost_(settings.Q, settings.R, settings.Ra, 0),
        directionCost_(settings.alpha, settings.weight, 1),
        finalCost_(settings.Qf, 0),
        constraintAugmentedLagrangian_(settings.constraintSettings),
        dynamics_(settings.alpha) {
    problem_.dynamicsPtr = &dynamics_;
    problem_.cost.add(trackCost_);
    problem_.cost.add(directionCost_);
    problem_.finalCost.add(finalCost_);
    // 注册 state-input 不等式增广拉格朗日项：
    // term 0: 推力锥约束 + 总加速度椭球约束（2 维）
    // term 1: z 轴最小值约束（1 维）
    problem_.inequalityLagrangian.template set<0>(
        constraintAugmentedLagrangian_.constraintLagrangian());
    problem_.inequalityLagrangian.template set<1>(
        constraintAugmentedLagrangian_.zMinLagrangian());
  }
  ~ThrustVectorOptimalControlProblem() = default;

  Problem_t& problem() { return problem_; }
  const Problem_t& problem() const { return problem_; }

 protected:
  TrackCost_t trackCost_;
  DirectionCost_t directionCost_;
  FinalCost_t finalCost_;
  ConstraintAugmentedLagrangian_t constraintAugmentedLagrangian_;
  ThrustVectorDynamicSystem_t dynamics_;
  Problem_t problem_;
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorILQR
    : public ThrustVectorOptimalControlProblem<Scalar, PredictLength> {
 public:
  using Descriptor_t = iLQRDescriptor<
      Scalar,
      TranscriptionConfig<Dimensions<STATE_DIM, INPUT_DIM>,
                          Horizon<PredictLength>, DiscreteDynamics>,
      ThrustVectorConstraintConfig<Scalar>>;
  using Solver_t = iLQR<Descriptor_t>;
  using HoverInitializer_t = HoverInitializer<Scalar>;
  using StateVector_t = typename Solver_t::StateVector_t;
  using InputVector_t = typename Solver_t::InputVector_t;
  using TimeTrajectory_t = typename Solver_t::TimeTrajectory_t;
  using StateTrajectory_t = typename Solver_t::StateTrajectory_t;
  using InputTrajectory_t = typename Solver_t::InputTrajectory_t;
  using ReferenceTrajectoryGenerator_t =
      ThrustVectorReferenceTrajectoryGenerator<Scalar>;

  explicit ThrustVectorILQR(const ThrustVectorILQRSettings<Scalar>& settings)
      : ThrustVectorOptimalControlProblem<Scalar, PredictLength>(
            settings.ocpSettings),
        alpha_(settings.ocpSettings.alpha),
        referenceTrajectoryAlpha_(
            settings.ocpSettings.referenceTrajectoryAlpha),
        initializer_(settings.ocpSettings.alpha),
        solver_(settings.ddpSettings, this->problem_, &initializer_) {
    assert(alpha_ >= Scalar(0));
    assert(alpha_ <= Scalar(1));

    for (int i = 0; i < 3; ++i) {
      assert(referenceTrajectoryAlpha_(i) >= Scalar(0));
      assert(referenceTrajectoryAlpha_(i) <= Scalar(1));
    }
  }

  ~ThrustVectorILQR() = default;
  ThrustVectorILQR(const ThrustVectorILQR& other) = delete;
  ThrustVectorILQR& operator=(const ThrustVectorILQR& rhs) = delete;
  ThrustVectorILQR(ThrustVectorILQR&& other) = delete;
  ThrustVectorILQR& operator=(ThrustVectorILQR&& rhs) = delete;

  Solver_t& solver() { return solver_; }
  ReferenceTrajectoryGenerator_t& referenceTrajectoryGenerator() {
    return referenceTrajectoryGenerator_;
  }
  const ReferenceTrajectoryGenerator_t& referenceTrajectoryGenerator() const {
    return referenceTrajectoryGenerator_;
  }
  static InputVector_t hoverInput() {
    return InputVector_t{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
  }

  void resetReferenceTrajectory(
      const Vector<Scalar, 3>& current_velocity,
      const Vector<Scalar, 3>& current_effective_acceleration) {
    (void)current_effective_acceleration;
    referenceTrajectoryGenerator_.reset(current_velocity);
  }

  void setDesireTrajectory(
      const Scalar current_time, const Vector<Scalar, 3>& vel_sp,
      const Vector<Scalar, 3>& current_velocity,
      const Vector<Scalar, 3>& command_acceleration_reference) {
    auto& targetTrajectory = solver_.targetTrajectory();

    if (!referenceTrajectoryGenerator_.initialized()) {
      referenceTrajectoryGenerator_.reset(current_velocity);
    }

    referenceTrajectoryGenerator_.generate(
        current_time, solver_.ddpSettings().timeStep, vel_sp,
        command_acceleration_reference, targetTrajectory,
        referenceTrajectoryAlpha_);
    referenceTrajectoryGenerator_.update(vel_sp, referenceTrajectoryAlpha_);
  }

 private:
  ReferenceTrajectoryGenerator_t referenceTrajectoryGenerator_;
  Scalar alpha_;
  Vector<Scalar, 3> referenceTrajectoryAlpha_;
  HoverInitializer_t initializer_;
  Solver_t solver_;
};

template <typename Scalar, size_t PredictLength>
inline ThrustVectorILQR<Scalar, PredictLength>
createThrustVectorFirstOrderLagProblem(
    const ThrustVectorILQRSettings<Scalar>& settings) {
  return ThrustVectorILQR<Scalar, PredictLength>(settings);
}
}  // namespace tvfol_constrain
