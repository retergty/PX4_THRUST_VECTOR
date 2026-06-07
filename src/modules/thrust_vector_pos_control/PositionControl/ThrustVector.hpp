#pragma once

#include <cmath>
#include <cstddef>
#include <iLQR/iLQR/iLQRCore.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <limits>

#include "iLQR/DDPSetting.hpp"
#include "iLQR/LinearAlgebraTypes.hpp"
#include "matrix/Vector3.hpp"
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * 该示例定义了一个最优控制问题，其中运动学建模的
 * 是一个矢量推力无人机，状态空间是6维
 * NED 全局坐标系三轴速度+上一拍机体系推力，输入是3维机体系推力
 * 同时添加余弦相似度的代价函数
 */

namespace thrust_vector {
static constexpr int STATE_DIM = 6;
static constexpr int INPUT_DIM = 3;
static constexpr float kGravity = 9.80665f;
template <typename Scalar, size_t PredictLength>
using ThrustVectorProblem = OptimalControlProblem<
    Scalar,
    TranscriptionConfig<Dimensions<STATE_DIM, INPUT_DIM>,
                        Horizon<PredictLength>, DiscreteDynamics>,
    ConstraintConfig<>>;

template <typename Scalar>
class ThrustVectorDynamicSystem final
    : public DiscreteSystemDynamicsBase<Scalar, STATE_DIM, INPUT_DIM> {
 public:
  using LinearApproximation_t =
      typename DiscreteSystemDynamicsBase<Scalar, STATE_DIM,
                                          INPUT_DIM>::LinearApproximation_t;

  struct PreCompCache {
    PreCompCache() {
      A.setZero();
      B.setZero();
      approximation.setZero();
      dt = 0;
      dirty = true;
      A.template topLeftCorner<3, 3>().setIdentity();
      B.template bottomRows<3>().setIdentity();
      approximation.dfdx = A;
    }
    Matrix<Scalar, STATE_DIM, STATE_DIM> A;
    Matrix<Scalar, STATE_DIM, INPUT_DIM> B;
    LinearApproximation_t approximation;
    Scalar dt;
    bool dirty;
  };

  ThrustVectorDynamicSystem() = default;
  ~ThrustVectorDynamicSystem() override = default;

  void updateCache(const Scalar dt) {
    if (cache_.dirty ||
        std::abs(cache_.dt - dt) > std::numeric_limits<Scalar>::epsilon()) {
      cache_.B.template topRows<3>() = dt * Matrix<Scalar, 3, 3>::Identity();
      cache_.dt = dt;
      cache_.dirty = false;
    }
  }

  LinearApproximation_t linearApproximation(Scalar t,
                                            const Vector<Scalar, STATE_DIM>& x,
                                            const Vector<Scalar, INPUT_DIM>& u,
                                            Scalar dt) override {
    LinearApproximation_t approximation;
    approximation.f = computeMap(t, x, u, dt);
    approximation.dfdx = cache_.A;
    approximation.dfdu = cache_.B;
    return approximation;
  }

  LinearApproximation_t deviationLinearApproximation(
      Scalar t, const Vector<Scalar, STATE_DIM>& x,
      const Vector<Scalar, INPUT_DIM>& u, Scalar dt) override {
    (void)t;
    (void)x;
    (void)u;

    updateCache(dt);
    cache_.approximation.dfdu = cache_.B;
    return cache_.approximation;
  }
  Vector<Scalar, STATE_DIM> computeMap(Scalar t,
                                       const Vector<Scalar, STATE_DIM>& x,
                                       const Vector<Scalar, INPUT_DIM>& u,
                                       Scalar dt) override {
    (void)t;

    Vector<Scalar, STATE_DIM> next_state;
    updateCache(dt);
    next_state.template head<3>() =
        x.template head<3>() + cache_.B.template topRows<3>() * u;
    next_state.template tail<3>() = u;

    return next_state;
  }

 private:
  PreCompCache cache_;
};

template <typename Scalar, int ArrayLength>
class ThrustVectorTrackCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorTrackCost(const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q,
                        const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& R,
                        int cost_number)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        Qv_(Q.template topLeftCorner<3, 3>()),
        R_(R) {
    approximation_.setZero();
    approximation_.dfdxx.template topLeftCorner<3, 3>() = Qv_;
    approximation_.dfduu = R_;
  }
  ~ThrustVectorTrackCost() override = default;

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectoy,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(indexAlpha, stateTrajectoy);
    const Vector<Scalar, INPUT_DIM> inputDeviation =
        input - interpolateInputReference(indexAlpha, inputTrajectory);
    const Vector<Scalar, 3> weightedVelocityDeviation = Qv_ * velocityDeviation;
    const Vector<Scalar, INPUT_DIM> weightedInputDeviation =
        R_ * inputDeviation;

    return Scalar(0.5) * velocityDeviation.dot(weightedVelocityDeviation) +
           Scalar(0.5) * inputDeviation.dot(weightedInputDeviation);
  }

  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
  getQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectoy,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    const auto indexAlpha =
        LinearInterpolation::timeSegment(time, timeTrajectory);
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(indexAlpha, stateTrajectoy);
    const Vector<Scalar, INPUT_DIM> inputDeviation =
        input - interpolateInputReference(indexAlpha, inputTrajectory);
    const Vector<Scalar, 3> weightedVelocityDeviation = Qv_ * velocityDeviation;
    const Vector<Scalar, INPUT_DIM> weightedInputDeviation =
        R_ * inputDeviation;

    approximation_.f =
        Scalar(0.5) * velocityDeviation.dot(weightedVelocityDeviation) +
        Scalar(0.5) * inputDeviation.dot(weightedInputDeviation);
    approximation_.dfdx.template head<3>() = weightedVelocityDeviation;
    approximation_.dfdu = weightedInputDeviation;
    return approximation_;
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

  Matrix<Scalar, 3, 3> Qv_;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> R_;
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
      approximation_;
};

template <typename Scalar, int ArrayLength>
class ThrustVectorTrackFinalCost final
    : public StateCost<Scalar, STATE_DIM, ArrayLength> {
 public:
  ThrustVectorTrackFinalCost(const Matrix<Scalar, STATE_DIM, STATE_DIM>& Q,
                             int cost_number)
      : StateCost<Scalar, STATE_DIM, ArrayLength>(cost_number),
        Qv_(Q.template topLeftCorner<3, 3>()) {
    approximation_.setZero();
    approximation_.dfdxx.template topLeftCorner<3, 3>() = Qv_;
  }
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

  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, 0>
  getQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory)
      override {
    const Vector<Scalar, 3> velocityDeviation =
        state.template head<3>() -
        interpolateVelocityReference(time, timeTrajectory, stateTrajectory);

    approximation_.f =
        Scalar(0.5) * velocityDeviation.dot(Qv_ * velocityDeviation);
    approximation_.dfdx.template head<3>() = Qv_ * velocityDeviation;
    return approximation_;
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
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, 0> approximation_;
};

template <typename Scalar, int ArrayLength>
class ThrustVectorInputRateCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
 public:
  ThrustVectorInputRateCost(const Matrix<Scalar, INPUT_DIM, INPUT_DIM>& S,
                      int cost_number)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        S_(S) {
    approximation_.setZero();
    approximation_.dfdxx.template slice<3, 3>(3, 3) = S_;
    approximation_.dfduu = S_;
    approximation_.dfdux.template slice<3, 3>(0, 3) = -S_;
  }

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectoy,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    (void)time;
    (void)timeTrajectory;
    (void)stateTrajectoy;
    (void)inputTrajectory;

    const Vector<Scalar, INPUT_DIM> inputRate =
        input - state.template tail<INPUT_DIM>();
    return Scalar(0.5) * inputRate.dot(S_ * inputRate);
  }

  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
  getQuadraticApproximation(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectory,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    (void)time;
    (void)timeTrajectory;
    (void)stateTrajectory;
    (void)inputTrajectory;

    const Vector<Scalar, INPUT_DIM> inputRate =
        input - state.template tail<INPUT_DIM>();
    const Vector<Scalar, INPUT_DIM> weightedInputRate = S_ * inputRate;
    approximation_.f = Scalar(0.5) * inputRate.dot(weightedInputRate);
    approximation_.dfdx.template tail<INPUT_DIM>() = -weightedInputRate;
    approximation_.dfdu = weightedInputRate;
    return approximation_;
  }

 private:
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> S_;
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
      approximation_;
};

template <typename Scalar, int ArrayLength>
class ThrustDirectionChangeCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
  /** @brief 获取代价值。 */
 public:
  static constexpr Scalar epsilon = Scalar(1e-4);
  static constexpr Scalar MinThrustAccelerationForDirection = Scalar(1e-2);

  explicit ThrustDirectionChangeCost(Scalar weight = 1, int cost_number = 0)
      : StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength>(cost_number),
        weight_(weight) {
    approximation_.setZero();
  }

  Scalar getValue(
      Scalar time, const Vector<Scalar, STATE_DIM>& state,
      const Vector<Scalar, INPUT_DIM>& input,
      const std::array<Scalar, ArrayLength>& timeTrajectory,
      const std::array<Vector<Scalar, STATE_DIM>, ArrayLength>& stateTrajectoy,
      const std::array<Vector<Scalar, INPUT_DIM>, ArrayLength>& inputTrajectory)
      override {
    (void)time;
    (void)timeTrajectory;
    (void)inputTrajectory;
    (void)stateTrajectoy;

    const Vector<Scalar, 3> lastThrustAcceleration =
        state.template tail<3>() - gravityVector();
    const Vector<Scalar, 3> thrustAcceleration = input - gravityVector();
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

  /** @brief 获取代价的二次近似（状态-输入）。 */
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
  getQuadraticApproximation(
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
        state.template tail<3>() - gravityVector();
    const Vector<Scalar, 3> thrustAcceleration = input - gravityVector();
    const Scalar lastThrustAccelerationNorm =
        std::sqrt(lastThrustAcceleration.dot(lastThrustAcceleration) + epsilon);
    const Scalar thrustAccelerationNorm =
        std::sqrt(thrustAcceleration.dot(thrustAcceleration) + epsilon);
    const Vector<Scalar, 3> lastThrustDirection =
        lastThrustAcceleration / lastThrustAccelerationNorm;
    const Vector<Scalar, 3> thrustDirection =
        thrustAcceleration / thrustAccelerationNorm;
    const Vector<Scalar, 3> rk = thrustDirection - lastThrustDirection;

    const Matrix<Scalar, 3, 3> identity = Matrix<Scalar, 3, 3>::Identity();
    const Matrix<Scalar, 3, 3> lastThrustAccelerationJacobian =
        identity / lastThrustAccelerationNorm -
        (lastThrustAcceleration * lastThrustAcceleration.transpose()) /
            (lastThrustAccelerationNorm * lastThrustAccelerationNorm *
             lastThrustAccelerationNorm);
    const Matrix<Scalar, 3, 3> thrustAccelerationJacobian =
        identity / thrustAccelerationNorm -
        (thrustAcceleration * thrustAcceleration.transpose()) /
            (thrustAccelerationNorm * thrustAccelerationNorm *
             thrustAccelerationNorm);

    const Scalar gate =
        lowAccelerationGate(lastThrustAcceleration, thrustAcceleration);
    const Scalar effectiveWeight = gate * weight_;

    approximation_.f = Scalar(0.5) * effectiveWeight * rk.dot(rk);
    approximation_.dfdx.template tail<3>() =
        -effectiveWeight * lastThrustAccelerationJacobian.transpose() * rk;
    approximation_.dfdu =
        effectiveWeight * thrustAccelerationJacobian.transpose() * rk;
    approximation_.dfdxx.template slice<3, 3>(3, 3) =
        effectiveWeight * lastThrustAccelerationJacobian.transpose() *
        lastThrustAccelerationJacobian;
    approximation_.dfduu = effectiveWeight *
                           thrustAccelerationJacobian.transpose() *
                           thrustAccelerationJacobian;
    approximation_.dfdux.template slice<3, 3>(0, 3) =
        -effectiveWeight * thrustAccelerationJacobian.transpose() *
        lastThrustAccelerationJacobian;

    return approximation_;
  }

 private:
  static Vector<Scalar, 3> gravityVector() {
    return Vector<Scalar, 3>{Scalar(0), Scalar(0), kGravity};
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

  Scalar weight_;
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
      approximation_;
};

struct ThrustVectorReferenceTrajectorySettings {
  matrix::Vector3f max_velocity{0.f, 0.f, 0.f};
  matrix::Vector3f max_acceleration{0.f, 0.f, 0.f};
  matrix::Vector3f max_jerk{0.f, 0.f, 0.f};
  bool synchronize_axes{false};
};

class ThrustVectorReferenceTrajectoryGenerator {
 public:
  ThrustVectorReferenceTrajectoryGenerator() = default;
  explicit ThrustVectorReferenceTrajectoryGenerator(
      const ThrustVectorReferenceTrajectorySettings& settings) {
    setSettings(settings);
  }

  void setSettings(const ThrustVectorReferenceTrajectorySettings& settings) {
    settings_ = settings;
    setLimits(settings_.max_velocity, settings_.max_acceleration,
              settings_.max_jerk);
  }

  const ThrustVectorReferenceTrajectorySettings& settings() const {
    return settings_;
  }

  bool initialized() const { return initialized_; }

  bool hasValidLimits() const {
    return isPositive(settings_.max_velocity) &&
           isPositive(settings_.max_acceleration) &&
           isPositive(settings_.max_jerk);
  }

  void reset(const matrix::Vector3f& acceleration,
             const matrix::Vector3f& velocity) {
    for (size_t axis = 0; axis < 3; ++axis) {
      smooth_[axis].reset(acceleration(axis), velocity(axis), 0.f);
    }

    initialized_ = true;
  }

  void setVelocityLimits(const matrix::Vector3f& max_velocity) {
    settings_.max_velocity = max_velocity;

    for (size_t axis = 0; axis < 3; ++axis) {
      smooth_[axis].setMaxVel(max_velocity(axis));
    }
  }

  void setAccelerationLimits(const matrix::Vector3f& max_acceleration) {
    settings_.max_acceleration = max_acceleration;

    for (size_t axis = 0; axis < 3; ++axis) {
      smooth_[axis].setMaxAccel(max_acceleration(axis));
    }
  }

  void setJerkLimits(const matrix::Vector3f& max_jerk) {
    settings_.max_jerk = max_jerk;

    for (size_t axis = 0; axis < 3; ++axis) {
      smooth_[axis].setMaxJerk(max_jerk(axis));
    }
  }

  void setLimits(const matrix::Vector3f& max_velocity,
                 const matrix::Vector3f& max_acceleration,
                 const matrix::Vector3f& max_jerk) {
    setVelocityLimits(max_velocity);
    setAccelerationLimits(max_acceleration);
    setJerkLimits(max_jerk);
  }

  void update(const matrix::Vector3f& velocity_setpoint, const float dt,
              const bool synchronize_axes) {
    updateSmoothers(smooth_, velocity_setpoint, dt, synchronize_axes);
  }

  void update(const matrix::Vector3f& velocity_setpoint, const float dt) {
    update(velocity_setpoint, dt, settings_.synchronize_axes);
  }

  matrix::Vector3f currentVelocity() const { return velocity(smooth_); }
  matrix::Vector3f currentAcceleration() const { return acceleration(smooth_); }
  matrix::Vector3f currentJerk() const { return jerk(smooth_); }

  template <typename TargetTrajectory>
  void generate(const float current_time, const float time_step,
                const matrix::Vector3f& velocity_setpoint,
                const matrix::Vector3f& last_input,
                TargetTrajectory& targetTrajectory,
                const bool synchronize_axes) const {
    VelocitySmoothing preview[3] = {smooth_[0], smooth_[1], smooth_[2]};

    for (size_t i = 0; i < targetTrajectory.timeTrajectory.size(); ++i) {
      if (i > 0) {
        updateSmoothers(preview, velocity_setpoint, time_step,
                        synchronize_axes);
      }

      targetTrajectory.timeTrajectory[i] =
          current_time + static_cast<float>(i) * time_step;
      targetTrajectory.stateTrajectory[i].setZero();
      targetTrajectory.stateTrajectory[i].template head<3>() =
          velocity(preview);
    }

    for (size_t i = 0; i + 1 < targetTrajectory.inputTrajectory.size(); ++i) {
      targetTrajectory.inputTrajectory[i] =
          (targetTrajectory.stateTrajectory[i + 1].template head<3>() -
           targetTrajectory.stateTrajectory[i].template head<3>()) /
          time_step;
    }

    if (targetTrajectory.inputTrajectory.size() > 1) {
      targetTrajectory.inputTrajectory.back() =
          targetTrajectory
              .inputTrajectory[targetTrajectory.inputTrajectory.size() - 2];
    }

    for (size_t i = 0; i < targetTrajectory.stateTrajectory.size(); ++i) {
      if (i == 0) {
        targetTrajectory.stateTrajectory[i].template tail<3>() = last_input;
      } else {
        targetTrajectory.stateTrajectory[i].template tail<3>() =
            targetTrajectory.inputTrajectory[i - 1];
      }
    }
  }

  template <typename TargetTrajectory>
  void generate(const float current_time, const float time_step,
                const matrix::Vector3f& velocity_setpoint,
                const matrix::Vector3f& last_input,
                TargetTrajectory& targetTrajectory) const {
    generate(current_time, time_step, velocity_setpoint, last_input,
             targetTrajectory, settings_.synchronize_axes);
  }

 private:
  static void updateSmoothers(VelocitySmoothing (&smooth)[3],
                              const matrix::Vector3f& velocity_setpoint,
                              const float dt, const bool synchronize_axes) {
    for (size_t axis = 0; axis < 3; ++axis) {
      smooth[axis].updateDurations(velocity_setpoint(axis));
    }

    if (synchronize_axes) {
      VelocitySmoothing::timeSynchronization(smooth, 3);
    }

    for (size_t axis = 0; axis < 3; ++axis) {
      smooth[axis].updateTraj(dt);
    }
  }

  static matrix::Vector3f velocity(const VelocitySmoothing (&smooth)[3]) {
    return matrix::Vector3f{smooth[0].getCurrentVelocity(),
                            smooth[1].getCurrentVelocity(),
                            smooth[2].getCurrentVelocity()};
  }

  static matrix::Vector3f acceleration(const VelocitySmoothing (&smooth)[3]) {
    return matrix::Vector3f{smooth[0].getCurrentAcceleration(),
                            smooth[1].getCurrentAcceleration(),
                            smooth[2].getCurrentAcceleration()};
  }

  static matrix::Vector3f jerk(const VelocitySmoothing (&smooth)[3]) {
    return matrix::Vector3f{smooth[0].getCurrentJerk(),
                            smooth[1].getCurrentJerk(),
                            smooth[2].getCurrentJerk()};
  }

  static bool isPositive(const matrix::Vector3f& value) {
    return value(0) > std::numeric_limits<float>::epsilon() &&
           value(1) > std::numeric_limits<float>::epsilon() &&
           value(2) > std::numeric_limits<float>::epsilon();
  }

  ThrustVectorReferenceTrajectorySettings settings_;
  VelocitySmoothing smooth_[3];
  bool initialized_{false};
};

template <typename Scalar>
struct ThrustVectorOCPSettings {
  // 权重
  Matrix<Scalar, STATE_DIM, STATE_DIM> Q;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> R;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> S;
  Matrix<Scalar, STATE_DIM, STATE_DIM> Qf;
  Scalar weight;
};

template <typename Scalar>
struct ThrustVectorILQRSettings {
  DDPSettings<Scalar> ddpSettings;

  ThrustVectorOCPSettings<Scalar> ocpSettings;

  ThrustVectorReferenceTrajectorySettings referenceTrajectorySettings;
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorOptimalControlProblem {
 public:
  using Problem_t = ThrustVectorProblem<Scalar, PredictLength>;
  using TrackCost_t =
      ThrustVectorTrackCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using DirectionCost_t =
      ThrustDirectionChangeCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using InputRateCost_t = ThrustVectorInputRateCost<Scalar,static_cast<int>(PredictLength + 1)>;
  using FinalCost_t =
      ThrustVectorTrackFinalCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using ThrustVectorDynamicSystem_t = ThrustVectorDynamicSystem<Scalar>;

  ThrustVectorOptimalControlProblem(
      const ThrustVectorOCPSettings<Scalar>& settings)
      : trackCost_(settings.Q, settings.R, 0),
        inputRateCost_(settings.S, 1),
        directionCost_(settings.weight, 2),
        finalCost_(settings.Qf, 0) {
    problem_.dynamicsPtr = &dynamics_;
    problem_.cost.add(trackCost_);
    problem_.cost.add(directionCost_);
    problem_.cost.add(inputRateCost_);
    problem_.finalCost.add(finalCost_);
  }
  ~ThrustVectorOptimalControlProblem() = default;

 protected:
  TrackCost_t trackCost_;
  InputRateCost_t inputRateCost_;
  DirectionCost_t directionCost_;
  FinalCost_t finalCost_;
  ThrustVectorDynamicSystem_t dynamics_;
  Problem_t problem_;
};

template <typename Scalar>
class HoverInitializer final
    : public Initializer<Scalar, STATE_DIM, INPUT_DIM> {
 public:
  HoverInitializer() {};
  void compute(const Scalar time, const Vector<Scalar, STATE_DIM>& state,
               const Scalar nextTime, Vector<Scalar, INPUT_DIM>& input,
               Vector<Scalar, STATE_DIM>& nextState) override {
    (void)time;
    const Scalar dt = nextTime - time;
    input = hoverInput();

    nextState.template head<3>() = state.template head<3>() + dt * input;
    nextState.template tail<3>() = input;
  }
  static Vector<Scalar, INPUT_DIM> hoverInput() {
    return Vector<Scalar, INPUT_DIM>{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
  }
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorILQR
    : public ThrustVectorOptimalControlProblem<Scalar, PredictLength> {
 public:
  using Descriptor_t = iLQRDescriptor<
      Scalar, TranscriptionConfig<Dimensions<STATE_DIM, INPUT_DIM>,
                                  Horizon<PredictLength>, DiscreteDynamics>>;
  using Solver_t = iLQR<Descriptor_t>;
  using HoverInitializer_t = HoverInitializer<Scalar>;
  using StateVector_t = typename Solver_t::StateVector_t;
  using InputVector_t = typename Solver_t::InputVector_t;
  using TimeTrajectory_t = typename Solver_t::TimeTrajectory_t;
  using StateTrajectory_t = typename Solver_t::StateTrajectory_t;
  using InputTrajectory_t = typename Solver_t::InputTrajectory_t;

  ThrustVectorILQR(ThrustVectorILQRSettings<Scalar>& settings)
      : ThrustVectorOptimalControlProblem<Scalar, PredictLength>(
            settings.ocpSettings),
        referenceTrajectoryGenerator_(settings.referenceTrajectorySettings),
        solver_(settings.ddpSettings, this->problem_, &initializer_) {}
  ~ThrustVectorILQR() = default;

  Solver_t& solver() { return solver_; }
  ThrustVectorReferenceTrajectoryGenerator& referenceTrajectoryGenerator() {
    return referenceTrajectoryGenerator_;
  }
  const ThrustVectorReferenceTrajectoryGenerator& referenceTrajectoryGenerator()
      const {
    return referenceTrajectoryGenerator_;
  }
  static InputVector_t hoverInput() {
    return InputVector_t{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
  }

  void setDesireTrajectory(const float current_time,
                           const matrix::Vector3f& vel_sp,
                           const matrix::Vector3f& current_velocity,
                           const matrix::Vector3f& last_input) {
    auto& targetTrajectory = solver_.targetTrajectory();

    if (referenceTrajectoryGenerator_.hasValidLimits()) {
      const float time_step = solver_.ddpSettings().timeStep;

      if (!referenceTrajectoryGenerator_.initialized()) {
        referenceTrajectoryGenerator_.reset(last_input, current_velocity);
      }

      referenceTrajectoryGenerator_.generate(current_time, time_step, vel_sp,
                                             last_input, targetTrajectory);
      referenceTrajectoryGenerator_.update(vel_sp, time_step);
      return;
    }

    for (size_t i = 0; i < PredictLength + 1; ++i) {
      targetTrajectory.timeTrajectory[i] =
          current_time + static_cast<float>(i) * solver_.ddpSettings().timeStep;
      targetTrajectory.stateTrajectory[i].setZero();
      targetTrajectory.stateTrajectory[i].template head<3>() = vel_sp;
      if (i == 0) {
        targetTrajectory.stateTrajectory[i].template tail<3>() = last_input;
      } else {
        targetTrajectory.stateTrajectory[i].template tail<3>() =
            targetTrajectory.inputTrajectory[i - 1];
      }
      targetTrajectory.inputTrajectory[i] = hoverInput();
    }
  }

 private:
  ThrustVectorReferenceTrajectoryGenerator referenceTrajectoryGenerator_;
  HoverInitializer_t initializer_;
  Solver_t solver_;
};

}  // namespace thrust_vector
