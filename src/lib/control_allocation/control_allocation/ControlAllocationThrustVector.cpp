/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationPseudoInverse.cpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationThrustVector.hpp"
#include <px4_platform_common/module.h>
ControlAllocationThrustVector::ControlAllocationThrustVector()
{
	for (int i = 0; i < 4; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);


		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);
		param_get(_param_handles[i].thrust_coef,&_thrust_vector_unit_geometry[i].thrust_coef);
	}
}

void
ControlAllocationThrustVector::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_qp_need_reinitialise = true;
}

void ControlAllocationThrustVector::print_status()
{
	ControlAllocation::print_status();
	PX4_INFO("Quad Programming. Size of ControlAllocationThrustVector: %d",
		 static_cast<int>(sizeof(ControlAllocationThrustVector)));
	PX4_INFO("Last QP Success %d", _last_qp_success);
	PX4_INFO("QP iteration count %d", _main_qp.GetIterationCount());
	PX4_INFO("QP optimal Vector");
	_main_qp.GetOptimalVector().print();

	// PX4_INFO("G factor (x,y,z): ");

	// for (int i = 0; i < 4; ++i) {
	// 	PX4_INFO_RAW("(%f, %f, %f), ", (double) _G(3 * i, 3 * i), (double)_G(3 * i + 1, 3 * i + 1),
	// 		     (double) _G(3 * i + 2, 3 * i + 2));
	// }

}
void
ControlAllocationThrustVector::updateQPState()
{
	if (_qp_need_reinitialise) {
		//init main QP
		{
			DiagnoalMatrix<float, 12> G;
			matrix::Vector<float, 12> g0;
			matrix::Matrix<float, 12, 6> CE = _effectiveness.transpose();
			matrix::Vector<float, 6> ce0 = -(_control_sp - _control_trim);
			matrix::Matrix<float, 12, 8> CI;
			matrix::Vector<float, 8> ci0;

			G.setIdentity();

			// for (int i = 0; i < 4; ++i) {
			// 	G(3 * i, 3 * i) /= FzFactor;
			// 	G(3 * i + 1, 3 * i + 1) /= FzFactor;
			// }

			g0.setZero();

			CI.setZero();

			// for (int i = 0; i < 4; ++i) {
			// 	CI(3 * i + 2, i) = -1;
			// }

			for (int i = 0; i < 4; ++i) {
				CI(3 * i, i + 4) = 1;
			}

			ci0.setZero();
			ci0(0) = ci0(1) = ci0(2) = ci0(3) = -QpLimits;
			ci0(4) = ci0(5) = ci0(6) = ci0(7) = 0;

			_main_qp.init(G, g0, CE, ce0, CI, ci0);

			_last_success_actuator_sp.setZero();
			_last_success_actuator_sp(0) = _last_success_actuator_sp(1) = _last_success_actuator_sp(2) = _last_success_actuator_sp(
							       3) = 0.05f;

			for (int i = 0; i < 4; ++i) {
				_thrust_vector_unit_geometry[i].rotation = _effectiveness.slice<3, 3>(3, 3 * i);
			}
		}
		_qp_need_reinitialise = false;
		_last_allocate_control_sp.setZero();
	}
}
void checkNumericStableAndSetMotorSetpoint(const matrix::Vector<float, 12> &force_vector,
		matrix::Vector3f motor_thr_sp[4])
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
void ControlAllocationThrustVector::computeMainQpSetpoint(const matrix::Vector<float, 12>
		&force_vector)
{
	matrix::Vector3f motor_thr_sp[4];

	checkNumericStableAndSetMotorSetpoint(force_vector, motor_thr_sp);

	for (size_t i = 0; i < 4; ++i) {
		const matrix::Vector3f &thr_sp = motor_thr_sp[i];

		ThrustVectorUnitSetpoint &unit_sp = _thrust_vector_unit_setpoint[i];

		unit_sp.InnerAngle = atan2f(thr_sp(1), -thr_sp(2));
		unit_sp.OuterAngle = atanf(thr_sp(0) / (thr_sp(2) * cosf(unit_sp.InnerAngle) -
							thr_sp(1) * sinf(unit_sp.InnerAngle)));
		unit_sp.Thrust = motor_thr_sp[i].length();
	}

	return ;
}

matrix::Vector<float, 12> ControlAllocationThrustVector::computeActuatorSetpoint() const
{
	matrix::Vector<float, 12> actuator_setpoint;

	for (size_t i = 0; i < 4; ++i) {
		const ThrustVectorUnitSetpoint &unit_sp = _thrust_vector_unit_setpoint[i];
		actuator_setpoint(i) = unit_sp.Thrust / _thrust_vector_unit_geometry[i].thrust_coef;
		actuator_setpoint(i + 4) = unit_sp.InnerAngle / M_PI_F * 2;
		actuator_setpoint(i + 8) = -unit_sp.OuterAngle / M_PI_F * 2;

		if (actuator_setpoint(i + 8) <= -0.03f) {
			//PX4_INFO("angle limit! %f", (double)(-_servo_angle[i](1)));
			actuator_setpoint(i + 8) = -0.03f;
		}
	}

	return actuator_setpoint;
}

void ControlAllocationThrustVector::updateMainQpSolver()
{
	// update CI
	matrix::Vector<float, 3> thrust_sp;
	thrust_sp(0) = _control_sp(3);
	thrust_sp(1) = _control_sp(4);
	thrust_sp(2) = _control_sp(5);

	float thrust_sp_length = thrust_sp.length();

	if (thrust_sp_length <= QpLimits + QpLimitsTol) {
		//set zero
		thrust_sp.setZero();

		// use default CI
		matrix::Matrix<float, 12, 8> &CI = _main_qp.GetCIRef();

		CI.setZero();

		for (int i = 0; i < 4; ++i) {
			CI(3 * i + 2, i) = -1;
		}

		for (int i = 0; i < 4; ++i) {
			CI(3 * i, i + 4) = 1;
		}

	} else {
		// normalize it
		thrust_sp /= thrust_sp_length;
		matrix::Matrix<float, 12, 8> &CI = _main_qp.GetCIRef();

		for (int i = 0; i < 4; ++i) {
			matrix::Matrix<float, 3, 1> cof = _thrust_vector_unit_geometry[i].rotation.transpose() * thrust_sp;
			CI.slice<3, 1>(3 * i, i) = cof;
		}
	}

	// // update G
	// for (int i = 0; i < 4; ++i) {
	// 	matrix::Vector<float, 3> thrust_vector_abs = _thrust_vector_unit_geometry[i].rotation.transpose() * thrust_sp;

	// 	float max_thrust_factor = 0;
	// 	int max_thrust_axis = 0;

	// 	for (int j = 0; j < 3; ++j) {
	// 		thrust_vector_abs(j) =  std::abs(thrust_vector_abs(j));

	// 		if (thrust_vector_abs(j) < 0.1f) { thrust_vector_abs(j) = 0.1f; }

	// 		if (thrust_vector_abs(j) > max_thrust_factor) {
	// 			max_thrust_factor = thrust_vector_abs(j);
	// 			max_thrust_axis = j;
	// 		}
	// 	}

	// 	// _G(3 * i, 3 * i) = max_thrust_factor / thrust_vector_abs(0);
	// 	// _G(3 * i + 1, 3 * i + 1) = max_thrust_factor / thrust_vector_abs(1);
	// 	// _G(3 * i + 2, 3 * i + 2) = max_thrust_factor / thrust_vector_abs(2);

	// 	// if (_G(3 * i, 3 * i) > 10) { _G(3 * i, 3 * i) = 10; }

	// 	// if (_G(3 * i + 1, 3 * i + 1) > 10) { _G(3 * i + 1, 3 * i + 1) = 10; }

	// 	// if (_G(3 * i + 2, 3 * i + 2) > 10) { _G(3 * i + 2, 3 * i + 2) = 10; }

	// 	_G(3 * i, 3 * i) = 1;
	// 	_G(3 * i + 1, 3 * i + 1) = 1;
	// 	_G(3 * i + 2, 3 * i + 2) = 1;

	// 	_G(3 * i + max_thrust_axis, 3 * i + max_thrust_axis) *= FzFactor;

	// }

	// _main_qp.updateScalarMatrix(_G);

	//update ce0
	matrix::Vector<float, 6> &Ce0 = _main_qp.GetCe0Ref();
	Ce0 = -(_control_sp - _control_trim);
}

void
ControlAllocationThrustVector::allocate()
{
	//Compute new gains if needed
	updateQPState();

	_prev_actuator_sp = _actuator_sp;

	// _control_sp(0) *= 2.5f;
	// _control_sp(1) *= 2.5f;
	// _control_sp(2) *= 2.0f;
	// _control_sp(3) *= 50.f;
	// _control_sp(4) *= 50.f;
	// _control_sp(5) *= 50.f;

	_control_sp(0) *= 2.1f;
	_control_sp(1) *= 2.1f;
	_control_sp(2) *= 1.5f;
	_control_sp(3) *= 50.f;
	_control_sp(4) *= 50.f;
	_control_sp(5) *= 50.f;

	//allocate
	updateMainQpSolver();

	if (_main_qp.Solve()) {
		_last_qp_success = true;
		computeMainQpSetpoint(_main_qp.GetOptimalVector());

		_last_allocate_control_sp = _control_sp;

	} else {
		_last_qp_success = false;
		_actuator_sp = _last_success_actuator_sp;
	}

	_actuator_sp = computeActuatorSetpoint();
}
