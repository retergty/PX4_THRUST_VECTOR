/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessMCThrustVector.hpp"

using namespace matrix;

ActuatorEffectivenessMCThrustVector::ActuatorEffectivenessMCThrustVector(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectiveness3DRotors::AxisConfiguration::Configurable)
{

}

bool
ActuatorEffectivenessMCThrustVector::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	bool matrix_need_updated = false;

	if (_attitude_setpoint_sub.update(&_att_sp)) {
		matrix_need_updated = _mc_rotors.computeServoAngle(Vector3f{_att_sp.thrust_body});

		servo_angle_s servo_angle_sp;
		servo_angle_sp.timestamp = hrt_absolute_time();
		_mc_rotors.getServoAngle().copyTo(servo_angle_sp.servo_angle);
		_servo_angle_pub.publish(servo_angle_sp);
	}

	if (external_update != EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		matrix_need_updated = true;
	}

	if (!matrix_need_updated) { return false; }

	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	return (rotors_added_successfully);
}
void ActuatorEffectivenessMCThrustVector::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	const Vector<float, 8> &servo_angle_sp = _mc_rotors.getServoAngle();

	for (size_t i = 0; i < 4; ++i) {
		actuator_sp(i + 4) = servo_angle_sp(2 * i) / M_PI_F * 2;
		actuator_sp(i + 8) = -servo_angle_sp(2 * i + 1) / M_PI_F * 2;
	}
}
