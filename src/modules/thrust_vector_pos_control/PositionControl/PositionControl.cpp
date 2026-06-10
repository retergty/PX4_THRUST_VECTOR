/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"

#include <drivers/drv_hrt.h>
#include <float.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#include "ControlMath.hpp"
#include "iLQR/DDPSetting.hpp"
#include "matrix/Matrix.hpp"
#include "matrix/Vector3.hpp"
#include "px4_platform_common/log.h"

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {
    0,  {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN,
    NAN};

PositionControl::PositionControl() {
  thrust_vector_first_order_lag::ThrustVectorOCPSettings<float>& ocp_settings =
      ilqr_settings.ocpSettings;
  ocp_settings.Q.setZero();
  ocp_settings.Q.template topLeftCorner<3, 3>().setIdentity();
  ocp_settings.Q *= 2.0f;
  ocp_settings.Q(2, 2) *= 2.5f;
  ocp_settings.R = float(1) * decltype(ocp_settings.R)::Identity();
  ocp_settings.Ra = float(0.4) * decltype(ocp_settings.Ra)::Identity();
  ocp_settings.Qf.setZero();
  ocp_settings.Qf.template topLeftCorner<3, 3>() =
      float(10.0) * matrix::Matrix<float, 3, 3>::Identity();
  ocp_settings.weight = 1;
  ocp_settings.alpha = 0.3;
  ocp_settings.referenceTrajectoryAlpha = matrix::Vector3f{0.4f, 0.4f, 0.6f};

  DDPSettings<float>& ddp_settings = ilqr_settings.ddpSettings;

  ddp_settings.timeStep = kTimeStep;
  ddp_settings.maxNumIterations = 3;

  setAccelerationLimits(5.f, 8.f, 8.f);

  _ilqr = new thrust_vector_first_order_lag::ThrustVectorILQR<float,
                                                              kPredictLength>(
      ilqr_settings);

  _state.setZero();
  _input.setZero();
  _acceleration.setZero();
}

void PositionControl::setVelocityLimits(const float vel_horizontal,
                                        const float vel_up,
                                        const float vel_down) {
  _lim_vel_horizontal = vel_horizontal;
  _lim_vel_up = vel_up;
  _lim_vel_down = vel_down;
}

void PositionControl::setAccelerationLimits(const float acc_horizontal,
                                            const float acc_up,
                                            const float acc_down) {
  _lim_acc_horizontal = math::max(acc_horizontal, 0.f);
  _lim_acc_up = math::max(acc_up, 0.f);
  _lim_acc_down = math::max(acc_down, 0.f);
}

void PositionControl::setAccelerationBiasEstimatorGains(
    const matrix::Vector3f& Ki, const matrix::Vector3f& leak,
    const matrix::Vector3f& alpha) {
  accel_bias_estimator_.setBiasIntegrateGain(Ki);
  accel_bias_estimator_.setBiasLeakGain(leak);
  accel_bias_estimator_.setLowPassAlpha(alpha);
}

void PositionControl::setAccelerationBiasLimits(const float bias_horizontal,
                                                const float bias_vertical) {
  const float horizontal_limit = math::max(bias_horizontal, 0.f);
  const float vertical_limit = math::max(bias_vertical, 0.f);

  accel_bias_estimator_.setLimits(
      matrix::Vector3f{horizontal_limit, horizontal_limit, vertical_limit},
      matrix::Vector3f{-horizontal_limit, -horizontal_limit, -vertical_limit});
}

void PositionControl::setThrustLimits(const float min, const float max) {
  // make sure there's always enough thrust vector length to infer the attitude
  _lim_thr_min = math::max(min, 10e-4f);
  _lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin) {
  _lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new) {
  // Given that the equation for thrust is T = a_sp * Th / g - Th
  // with a_sp = desired acceleration, Th = hover thrust and g = gravity
  // constant, we want to find the acceleration that needs to be added to the
  // integrator in order obtain the same thrust after replacing the current
  // hover thrust by the new one. T' = T => a_sp' * Th' / g - Th' = a_sp * Th /
  // g - Th so a_sp' = (a_sp - g) * Th / Th' + g we can then add a_sp' - a_sp to
  // the current integrator to absorb the effect of changing Th by Th'
  setHoverThrust(hover_thrust_new);
}

void PositionControl::setState(const PositionControlStates& states) {
  _time = states.time;
  _pos = states.position;
  _vel = states.velocity;
  _acceleration = states.acceleration;
  _yaw = states.yaw;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s& setpoint) {
  _pos_sp = Vector3f(setpoint.position);
  _vel_sp = Vector3f(setpoint.velocity);
  _acc_sp = Vector3f(setpoint.acceleration);
  _yaw_sp = setpoint.yaw;
  _yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::setPitchSetpoint(const float pitch_sp) {
  _pitch_sp = pitch_sp;
}

void PositionControl::setRollSetpoint(const float roll_sp) {
  _roll_sp = roll_sp;
}

bool PositionControl::update(const float dt) {
  bool valid = _inputValid();

  if (valid) {
    _positionControl();
    _velocityControl(dt);

    _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
    _yaw_sp = PX4_ISFINITE(_yaw_sp)
                  ? _yaw_sp
                  : _yaw;  // TODO: better way to disable yaw control
  }
  // There has to be a valid output acceleration and thrust setpoint otherwise
  // something went wrong
  return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl() {
  // P-position controller
  Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
  //  Position and feed-forward velocity setpoints or position states being NAN
  //  results in them not having an influence
  ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
  // make sure there are no NAN elements for further reference while
  // constraining
  ControlMath::setZeroIfNanVector3f(vel_sp_position);
  //  Constrain horizontal velocity by prioritizing the velocity component along
  //  the the desired position setpoint over the feed-forward term.
  _vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(),
                                          (_vel_sp - vel_sp_position).xy(),
                                          _lim_vel_horizontal);
  // Constrain velocity in z-direction.
  _vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt) {
  if (_vel_sp.isAllFinite() && _vel.isAllFinite() &&
      _acceleration.isAllFinite()) {
    const Vector3f velocity_error = _vel_sp - _vel;
    bool freeze_bias_axis[3] = {false, false, false};

    const float current_input_xy_norm =
        sqrtf(_input(0) * _input(0) + _input(1) * _input(1));

    if ((_lim_acc_horizontal > FLT_EPSILON) &&
        (current_input_xy_norm >= _lim_acc_horizontal - FLT_EPSILON)) {
      const float horizontal_error_projection =
          _input(0) * velocity_error(0) + _input(1) * velocity_error(1);

      if (horizontal_error_projection > 0.f) {
        freeze_bias_axis[0] = true;
        freeze_bias_axis[1] = true;
      }
    }

    if (((_input(2) <= -_lim_acc_up + FLT_EPSILON) &&
         (velocity_error(2) < 0.f)) ||
        ((_input(2) >= _lim_acc_down - FLT_EPSILON) &&
         (velocity_error(2) > 0.f))) {
      freeze_bias_axis[2] = true;
    }

    accel_bias_estimator_.update(dt, velocity_error, freeze_bias_axis);

    _ilqr->setDesireTrajectory(_time, _vel_sp, _vel,
                               accel_bias_estimator_.bias());
    _state.template head<3>() = _vel;
    _state.template segment<3>(3) = _acceleration;
    _state.template segment<3>(6) = _input;
    _ilqr->solver().run(_time, _state);

    const auto& primalSolution = _ilqr->solver().primalSolution();
    const Vector3f delta_input = primalSolution.inputTrajectory_.front();
    _input += delta_input;

    const float input_xy_norm =
        sqrtf(_input(0) * _input(0) + _input(1) * _input(1));
    if ((_lim_acc_horizontal > FLT_EPSILON) &&
        (input_xy_norm > _lim_acc_horizontal)) {
      const float scale = _lim_acc_horizontal / input_xy_norm;
      _input(0) *= scale;
      _input(1) *= scale;
    }

    _input(2) = math::constrain(_input(2), -_lim_acc_up, _lim_acc_down);
    ControlMath::addIfNotNanVector3f(_acc_sp, _input);
  }
  _accelerationControl();
}

void PositionControl::_accelerationControl() {
  matrix::Vector3f acc_thr_sp = _acc_sp;
  acc_thr_sp(2) += -CONSTANTS_ONE_G;

  _thr_sp = acc_thr_sp * (_hover_thrust / CONSTANTS_ONE_G);
  _thr_sp(2) = math::min(_thr_sp(2), -_lim_thr_min);
}

bool PositionControl::_inputValid() {
  bool valid = true;

  // Every axis x, y, z needs to have some setpoint
  for (int i = 0; i <= 2; i++) {
    valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) ||
                      PX4_ISFINITE(_acc_sp(i)));
  }

  // x and y input setpoints always have to come in pairs
  valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
  valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
  valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

  // For each controlled state the estimate has to be valid
  for (int i = 0; i <= 2; i++) {
    if (PX4_ISFINITE(_pos_sp(i))) {
      valid = valid && PX4_ISFINITE(_pos(i));
    }

    if (PX4_ISFINITE(_vel_sp(i))) {
      valid = valid && PX4_ISFINITE(_vel(i));
    }

    if (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i))) {
      valid = valid && PX4_ISFINITE(_acceleration(i));
    }
  }

  return valid;
}
void PositionControl::getLocalPositionSetpoint(
    vehicle_local_position_setpoint_s& local_position_setpoint) const {
  local_position_setpoint.x = _pos_sp(0);
  local_position_setpoint.y = _pos_sp(1);
  local_position_setpoint.z = _pos_sp(2);
  local_position_setpoint.yaw = _yaw_sp;
  local_position_setpoint.yawspeed = _yawspeed_sp;
  local_position_setpoint.vx = _vel_sp(0);
  local_position_setpoint.vy = _vel_sp(1);
  local_position_setpoint.vz = _vel_sp(2);
  _acc_sp.copyTo(local_position_setpoint.acceleration);
  _thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(
    vehicle_attitude_setpoint_s& attitude_setpoint) const {
  ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, _roll_sp, _pitch_sp,
                                attitude_setpoint);

  if (std::isnan(_pitch_sp)) {
    attitude_setpoint.thrust_body[0] = 0.f;
  }

  if (std::isnan(_roll_sp)) {
    attitude_setpoint.thrust_body[1] = 0.f;
  }

  attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
  //       std::cout << "thrust_body: " << attitude_setpoint.thrust_body[0] << "
  //       "
  //       << attitude_setpoint.thrust_body[1] << " " <<
  //       attitude_setpoint.thrust_body[2]
  //       << " " << std::endl;
}

int PositionControl::print_status() {
  PX4_INFO("iLQR size: %ld", sizeof(decltype(*_ilqr)));
  PX4_INFO("iLQR avarage iteration: %f",(double)_ilqr->solver().averageNumIterations());
  return 0;
}
