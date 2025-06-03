/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectiveness3DRotors.cpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectiveness3DRotors.hpp"
#include <lib/matrix/matrix/math.hpp>
using namespace matrix;

ActuatorEffectiveness3DRotors::ActuatorEffectiveness3DRotors(ModuleParams *parent, AxisConfiguration axis_config)
	: ModuleParams(parent), _axis_config(axis_config)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

	}

	_motor_q[0] = AxisAnglef(Vector3f(0, 0, 1), -M_PI / 4);
	_motor_q[1] = AxisAnglef(Vector3f(0, 0, 1), M_PI / 4 + M_PI / 2);
	_motor_q[2] = AxisAnglef(Vector3f(0, 0, 1), -M_PI / 4 - M_PI / 2);
	_motor_q[3] = AxisAnglef(Vector3f(0, 0, 1), M_PI / 4);

	Dcmf motor_R[4] {_motor_q[0], _motor_q[1], _motor_q[2], _motor_q[3]};
	Matrix<float, 3, 12> eff_mat;

	for (size_t i = 0; i < 4; ++i) {
		eff_mat.slice<3, 3>(0, i * 3) = motor_R[i];
	}

	geninv(eff_mat, _eff_mat_inv);

	for (size_t i = 0; i < 4; ++i) {
		_servo_angle[i](0) = _servo_angle[i](1) = 0;
	}

	updateParams();
}

bool ActuatorEffectiveness3DRotors::computeServoAngle(const Vector3f &thrust_body)
{
	Vector<float, 12> x = _eff_mat_inv * thrust_body;

	Vector3f motor_thr_sp[4];

	for (size_t i = 0; i < 4; ++i) {
		motor_thr_sp[i](0) = x(i * 3);
		motor_thr_sp[i](1) = x(i * 3 + 1);
		motor_thr_sp[i](2) = x(i * 3 + 2);
	}

	bool servo_angle_change = false;

	Vector<float, 2> servo_angle[4];

	for (size_t i = 0; i < 4; ++i) {
		servo_angle[i](0) = atanf(-motor_thr_sp[i](1) / motor_thr_sp[i](2));
		servo_angle[i](1) =  atanf(motor_thr_sp[i](0) /
					   (motor_thr_sp[i](2) * cosf(servo_angle[i](0)) - motor_thr_sp[i](1) * sinf(servo_angle[i](0))));
	}

	for (size_t i = 0; i < 4; ++i) {
		if (abs(_servo_angle[i](0) - servo_angle[i](0)) > 1e-4f) {
			servo_angle_change = true;
			break;

		} else if (abs(_servo_angle[i](1) - servo_angle[i](1)) > 1e-4f) {
			servo_angle_change = true;
			break;
		}
	}

	if (servo_angle_change) {
		for (size_t i = 0; i < 4; ++i) {
			_servo_angle[i] = servo_angle[i];
		}
	}

	return servo_angle_change;
}

void ActuatorEffectiveness3DRotors::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].initial_axis;

		switch (_axis_config) {
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);

		_geometry.rotors[i].tilt_index = -1;
	}
}

bool
ActuatorEffectiveness3DRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	for (int i = 0; i < _geometry.num_rotors; i++) {
		// Get correspont servo angle
		float roll_angle = _servo_angle[i](0);
		float pitch_angle = _servo_angle[i](1);

		Vector3f &axis = _geometry.rotors[i].axis;
		const Vector3f &initial_axis = _geometry.rotors[i].initial_axis;
		Quaternion<float> roll_q{AxisAnglef(Vector3f(1, 0, 0), roll_angle)};
		Quaternion<float> pitch_q{AxisAnglef(Vector3f(0, 1, 0), pitch_angle)};
		Quaternion<float> q = _motor_q[i] * roll_q * pitch_q;
		axis = q.rotateVector(initial_axis).normalized();
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_actuators);
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_actuators);
	return true;
}

int
ActuatorEffectiveness3DRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {
		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (geometry.propeller_torque_disabled) {
			km = 0.f;
		}

		if (geometry.propeller_torque_disabled_non_upwards) {
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}

		if (geometry.yaw_by_differential_thrust_disabled) {
			// set yaw effectiveness to 0 if yaw is controlled by other means (e.g. tilts)
			effectiveness(2, i + actuator_start_index) = 0.f;
		}

		if (geometry.three_dimensional_thrust_disabled) {
			// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
			// pass the vector magnitude as z-component, plus the collective tilt. Passing 3D thrust plus tilt is not feasible as they
			// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
			// by the allocator directly.

			effectiveness(0 + 3, i + actuator_start_index) = 0.f;
			effectiveness(1 + 3, i + actuator_start_index) = 0.f;
			effectiveness(2 + 3, i + actuator_start_index) = -ct;
		}
	}

	return num_actuators;
}

bool
ActuatorEffectiveness3DRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}
