#pragma once

#include <cmath>
#include <cstddef>
#include <iLQR/iLQR/iLQRCore.hpp>
#include <limits>

#include "iLQR/DDPSetting.hpp"
#include "iLQR/LinearAlgebraTypes.hpp"

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
      bias.setZero();
      approximation.setZero();
      dt = 0;
      dirty = true;
      A.template topLeftCorner<3, 3>().setIdentity();
      B.template bottomRows<3>().setIdentity();
      approximation.dfdx = A;
    }
    Matrix<Scalar, STATE_DIM, STATE_DIM> A;
    Matrix<Scalar, STATE_DIM, INPUT_DIM> B;
    Vector<Scalar, STATE_DIM> bias;
    LinearApproximation_t approximation;
    Scalar dt;
    bool dirty;
  };

  ThrustVectorDynamicSystem(const Scalar mass) : mass_(mass) {
    rotB2w_.setIdentity();
  }
  ~ThrustVectorDynamicSystem() override = default;

  void updateRotationMatrix(const Matrix<Scalar, 3, 3>& rotation) {
    rotB2w_ = rotation;
    cache_.dirty = true;
  }

  void updateCache(const Scalar dt) {
    if (cache_.dirty ||
        std::abs(cache_.dt - dt) > std::numeric_limits<Scalar>::epsilon()) {
      cache_.B.template topRows<3>() = (dt / mass_) * rotB2w_;
      cache_.bias(2) = dt * Scalar(kGravity);
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
    next_state.template head<3>() = x.template head<3>() +
                                    cache_.B.template topRows<3>() * u +
                                    cache_.bias.template head<3>();
    next_state.template tail<3>() = u;

    return next_state;
  }

 private:
  Scalar mass_;
  Matrix<Scalar, 3, 3> rotB2w_;
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
class ThrustDirectionChangeCost final
    : public StateInputCost<Scalar, STATE_DIM, INPUT_DIM, ArrayLength> {
  /** @brief 获取代价值。 */
 public:
  static constexpr Scalar epsilon = 1e-4;
  static constexpr Scalar MinThrustForDirection = 1e-2;

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

    const Vector<Scalar, 3> last_thr{state(3), state(4), state(5)};
    const Vector<Scalar, 3> thr = input;
    Vector<Scalar, 3> last_thr_dir =
        last_thr / std::sqrt(last_thr.dot(last_thr) + epsilon);
    Vector<Scalar, 3> thr_dir = thr / std::sqrt(thr.dot(thr) + epsilon);
    const Vector<Scalar, 3> rk = thr_dir - last_thr_dir;
    const Scalar gate = lowThrustGate(last_thr, thr);
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

    const Vector<Scalar, 3> last_thr{state(3), state(4), state(5)};
    const Vector<Scalar, 3> thr = input;
    const Scalar last_thr_norm = std::sqrt(last_thr.dot(last_thr) + epsilon);
    const Scalar thr_norm = std::sqrt(thr.dot(thr) + epsilon);
    const Vector<Scalar, 3> last_thr_dir = last_thr / last_thr_norm;
    const Vector<Scalar, 3> thr_dir = thr / thr_norm;
    const Vector<Scalar, 3> rk = thr_dir - last_thr_dir;

    const Matrix<Scalar, 3, 3> identity = Matrix<Scalar, 3, 3>::Identity();
    const Matrix<Scalar, 3, 3> last_thr_jacobian =
        identity / last_thr_norm -
        (last_thr * last_thr.transpose()) /
            (last_thr_norm * last_thr_norm * last_thr_norm);
    const Matrix<Scalar, 3, 3> thr_jacobian =
        identity / thr_norm -
        (thr * thr.transpose()) / (thr_norm * thr_norm * thr_norm);

    const Scalar gate = lowThrustGate(last_thr, thr);
    const Scalar effectiveWeight = gate * weight_;

    approximation_.f = Scalar(0.5) * effectiveWeight * rk.dot(rk);
    approximation_.dfdx.template tail<3>() =
        -effectiveWeight * last_thr_jacobian.transpose() * rk;
    approximation_.dfdu = effectiveWeight * thr_jacobian.transpose() * rk;
    approximation_.dfdxx.template slice<3, 3>(3, 3) =
        effectiveWeight * last_thr_jacobian.transpose() * last_thr_jacobian;
    approximation_.dfduu =
        effectiveWeight * thr_jacobian.transpose() * thr_jacobian;
    approximation_.dfdux.template slice<3, 3>(0, 3) =
        -effectiveWeight * thr_jacobian.transpose() * last_thr_jacobian;

    return approximation_;
  }

 private:
  static Scalar thrustGateSigma(const Vector<Scalar, 3>& thrust) {
    const Scalar thrustNormSquared = thrust.squaredNorm();
    const Scalar minThrustSquared =
        MinThrustForDirection * MinThrustForDirection;
    return thrustNormSquared / (thrustNormSquared + minThrustSquared);
  }

  static Scalar lowThrustGate(const Vector<Scalar, 3>& lastThrust,
                              const Vector<Scalar, 3>& currentThrust) {
    return thrustGateSigma(lastThrust) * thrustGateSigma(currentThrust);
  }

  Scalar weight_;
  ScalarFunctionQuadraticApproximation<Scalar, STATE_DIM, INPUT_DIM>
      approximation_;
};

template <typename Scalar>
struct ThrustVectorOCPSettings {
  // 机体质量
  Scalar mass;

  // 权重
  Matrix<Scalar, STATE_DIM, STATE_DIM> Q;
  Matrix<Scalar, INPUT_DIM, INPUT_DIM> R;
  Matrix<Scalar, STATE_DIM, STATE_DIM> Qf;
  Scalar weight;
};

template <typename Scalar>
struct ThrustVectorILQRSettings {
  DDPSettings<Scalar> ddpSettings;

  ThrustVectorOCPSettings<Scalar> ocpSettings;
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorOptimalControlProblem {
 public:
  using Problem_t = ThrustVectorProblem<Scalar, PredictLength>;
  using TrackCost_t =
      ThrustVectorTrackCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using DirectionCost_t =
      ThrustDirectionChangeCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using FinalCost_t =
      ThrustVectorTrackFinalCost<Scalar, static_cast<int>(PredictLength + 1)>;
  using ThrustVectorDynamicSystem_t = ThrustVectorDynamicSystem<Scalar>;

  ThrustVectorOptimalControlProblem(
      const ThrustVectorOCPSettings<Scalar>& settings)
      : trackCost_(settings.Q, settings.R, 0),
        directionCost_(settings.weight, 1),
        finalCost_(settings.Qf, 0),
        dynamics_(settings.mass) {
    problem_.dynamicsPtr = &dynamics_;
    problem_.cost.add(trackCost_);
    problem_.cost.add(directionCost_);
    problem_.finalCost.add(finalCost_);
  }
  ~ThrustVectorOptimalControlProblem() = default;

 protected:
  TrackCost_t trackCost_;
  DirectionCost_t directionCost_;
  FinalCost_t finalCost_;
  ThrustVectorDynamicSystem_t dynamics_;
  Problem_t problem_;
};

template <typename Scalar, size_t PredictLength>
class ThrustVectorILQR
    : public ThrustVectorOptimalControlProblem<Scalar, PredictLength> {
 public:
  using Descriptor_t = iLQRDescriptor<
      Scalar, TranscriptionConfig<Dimensions<STATE_DIM, INPUT_DIM>,
                                  Horizon<PredictLength>, DiscreteDynamics>>;
  using Solver_t = iLQR<Descriptor_t>;
  using StateVector_t = typename Solver_t::StateVector_t;
  using InputVector_t = typename Solver_t::InputVector_t;
  using TimeTrajectory_t = typename Solver_t::TimeTrajectory_t;
  using StateTrajectory_t = typename Solver_t::StateTrajectory_t;
  using InputTrajectory_t = typename Solver_t::InputTrajectory_t;

  ThrustVectorILQR(ThrustVectorILQRSettings<Scalar>& settings)
      : ThrustVectorOptimalControlProblem<Scalar, PredictLength>(
            settings.ocpSettings),
        initializer_(settings.ocpSettings.mass),
        solver_(settings.ddpSettings, this->problem_, &initializer_) {}
  ~ThrustVectorILQR() = default;

  Solver_t& solver() { return solver_; }
  void setRotationMatrix(const Matrix<Scalar, 3, 3>& rotB2w) {
    this->dynamics_.updateRotationMatrix(rotB2w);
  }

 private:
  class HoverInitializer final
      : public Initializer<Scalar, STATE_DIM, INPUT_DIM> {
   public:
    HoverInitializer(Scalar mass) : mass_(mass) {};
    void compute(const Scalar time, const StateVector_t& state,
                 const Scalar nextTime, InputVector_t& input,
                 StateVector_t& nextState) override {
      (void)time;
      const Scalar dt = nextTime - time;
      input = hoverInput();

      nextState.template head<3>() = state.template head<3>();
      nextState(2) += dt * (input(2) / mass_ + Scalar(kGravity));
      nextState.template tail<3>() = input;
    }
    InputVector_t hoverInput() {
      return InputVector_t{Scalar(0.0), Scalar(0.0), -mass_ * Scalar(kGravity)};
    }

   private:
    Scalar mass_;
  };
  HoverInitializer initializer_;
  Solver_t solver_;
};

}  // namespace thrust_vector
