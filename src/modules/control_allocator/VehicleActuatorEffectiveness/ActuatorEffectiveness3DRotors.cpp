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

	for (size_t i = 0; i < 4; ++i) {
		_servo_angle[i](0) = _servo_angle[i](1) = 0;
	}

	updateParams();
}
void ActuatorEffectiveness3DRotors::checkNumericStableAndSetMotorSetpoint(const matrix::Vector<float, 12> &force_vector,
		std::array<Vector3f, 4> &motor_thr_sp)
{
	for (size_t i = 0; i < 4; ++i) {
		if (force_vector(i * 3) < 0.f) {
			motor_thr_sp[i](0) = 0.f;

		} else {
			motor_thr_sp[i](0) = force_vector(i * 3);
		}
		motor_thr_sp[i](1) = force_vector(i * 3 + 1);
		motor_thr_sp[i](2) = force_vector(i * 3 + 2);
	}

}
matrix::Vector<float, 12>
ActuatorEffectiveness3DRotors::computeActuatorSetpoint(const matrix::Vector<float, 12> &force_vector)
{
	matrix::Vector<float, 12> actuator_setpoint;

	std::array<Vector3f, 4> motor_thr_sp;

	checkNumericStableAndSetMotorSetpoint(force_vector, motor_thr_sp);

	for (size_t i = 0; i < 4; ++i) {
		//_servo_angle[i](0) = atanf(-motor_thr_sp[i](1) / motor_thr_sp[i](2));
		_servo_angle[i](0) = atan2f(motor_thr_sp[i](1), -motor_thr_sp[i](2));
		_servo_angle[i](1) = atanf(motor_thr_sp[i](0) / (motor_thr_sp[i](2) * cosf(_servo_angle[i](0)) -
					   motor_thr_sp[i](1) * sinf(_servo_angle[i](0))));
		_rotor_speed[i] = motor_thr_sp[i].length();;
	}

	for (size_t i = 0; i < 4; ++i) {
		actuator_setpoint(i) = _rotor_speed[i];
		actuator_setpoint(i + 4) = _servo_angle[i](0) / M_PI_F * 2;
		actuator_setpoint(i + 8) = -_servo_angle[i](1) / M_PI_F * 2;

		if (actuator_setpoint(i + 8) <= -0.03f) {
			//PX4_INFO("angle limit! %f", (double)(-_servo_angle[i](1)));
			actuator_setpoint(i + 8) = -0.03f;
		}
	}

	return actuator_setpoint;
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

bool ActuatorEffectiveness3DRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators =
		computeEffectivenessMatrix(_geometry, configuration.effectiveness_matrices[configuration.selected_matrix],
					   configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_actuators);
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_actuators);
	return true;
}

int ActuatorEffectiveness3DRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness,
		int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {
		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float km = geometry.rotors[i].moment_ratio;
		//(void)km;

		Dcmf motor_R = _motor_q[i];

		effectiveness.slice<3, 3>(3, i * 3) = motor_R;

		for (int j = 0; j < 3; ++j) {
			const Vector3f axis = motor_R.col(j);
			effectiveness.slice<3, 1>(0, 3 * i + j) = position.cross(axis);//- km * axis;

			if (j == 2) {
				effectiveness.slice<3, 1>(0, 3 * i + j) += -km * axis;
			}
		}
	}

	return num_actuators;
}

bool ActuatorEffectiveness3DRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}
