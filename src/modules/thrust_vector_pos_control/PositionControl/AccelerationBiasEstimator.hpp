#pragma once
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>

#include "mathlib/math/Limits.hpp"
#include "matrix/Vector3.hpp"

template <typename Scalar>
class AccelerationBiasEstimator {
 public:
  using Vector3 = matrix::Vector<Scalar, 3>;

  AccelerationBiasEstimator() {
    accel_bias_.setZero();
    leak_.setZero();
    Ki_.setZero();
    upper_limit_.setZero();
    down_limit_.setZero();
  }
  void setBiasIntegrateGain(const Vector3& Ki) { Ki_ = Ki; }
  void setBiasLeakGain(const Vector3& leak) { leak_ = leak; }
  void setLimits(const Vector3& upper_limit, const Vector3& down_limit) {
    upper_limit_ = upper_limit;
    down_limit_ = down_limit;
  }
  void setLowPassAlpha(const Vector3& alpha) {
    for (int axis = 0; axis < 3; ++axis) {
      const Scalar alpha_constrain =
          math::constrain(alpha(axis), Scalar(0), Scalar(1));
      lpf_[axis].setAlpha(static_cast<float>(alpha_constrain));
    }
  }
  void resetAxis(int axis) {
    accel_bias_(axis) = Scalar(0);
    lpf_[axis].reset(accel_bias_(axis));
  }
  void reset() {
    for (int axis = 0; axis < 3; ++axis) {
      resetAxis(axis);
    }
  }
  const Vector3& bias() const { return accel_bias_; }
  void update(Scalar dt, const Vector3& velocity_error,
              const bool freeze_axis[3]) {
    for (int axis = 0; axis < 3; ++axis) {
      if (freeze_axis[axis]) {
        continue;
      }
      const Scalar e_v_filtered = lpf_[axis].update(velocity_error(axis));
      accel_bias_(axis) +=
          (Ki_(axis) * e_v_filtered - leak_(axis) * accel_bias_(axis)) * dt;
      accel_bias_(axis) = math::constrain(accel_bias_(axis), down_limit_(axis),
                                          upper_limit_(axis));
    }
  }

 private:
  Vector3 Ki_;
  Vector3 leak_;
  Vector3 upper_limit_;
  Vector3 down_limit_;
  AlphaFilter<Scalar> lpf_[3];
  Vector3 accel_bias_;
};
