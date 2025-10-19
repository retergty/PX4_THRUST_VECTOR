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
	PX4_INFO("QP iteration count %d", _qp.GetIterationCount());
	PX4_INFO("QP optimal Vector");
	_qp.GetOptimalVector().print();
}

void
ControlAllocationThrustVector::updateQPState()
{
	if (_qp_need_reinitialise) {
		matrix::Matrix<float, 12, 12> G;
		matrix::Vector<float, 12> g0;
		matrix::Matrix<float, 12, 6> CE = _effectiveness.transpose();
		matrix::Vector<float, 6> ce0 = -(_control_sp - _control_trim);
		matrix::Matrix<float, 12, 8> CI;
		matrix::Vector<float, 8> ci0;

		G.setIdentity();

		for (int i = 0; i < 4; ++i) {
			G(3 * i + 2, 3 * i + 2) *= FzFactor;
		}

		g0.setZero();

		CI.setZero();

		// for (int i = 0; i < 4; ++i) {
		// 	CI(3 * i + 2, i) = -1;
		// }

		for (int i = 0; i < 4; ++i) {
			CI(3 * i, i + 4) = 1;
		}

		ci0.setZero();
		ci0(0) = ci0(1) = ci0(2) = ci0(3) = -0.02;
		ci0(4) = ci0(5) = ci0(6) = ci0(7) = -0.01;

		_qp.init(G, g0, CE, ce0, CI, ci0);
		_qp_need_reinitialise = false;

		_last_success_actuator_sp.setZero();
		_last_success_actuator_sp(2) = _last_success_actuator_sp(5) = _last_success_actuator_sp(8) = _last_success_actuator_sp(11) = -0.05;

		for (int i = 0; i < 4; ++i) {
			_motor_R[i] = _effectiveness.slice<3, 3>(3, 3 * i);
		}

	}
}

void
ControlAllocationThrustVector::allocate()
{
	//Compute new gains if needed
	updateQPState();

	_prev_actuator_sp = _actuator_sp;

	//allocate
	_control_sp(0) /= 1.5f;
	_control_sp(1) /= 1.5f;
	_control_sp(2) /= 1.5f;
	_control_sp(3) *= 40;
	_control_sp(4) *= 40;
	_control_sp(5) *= 40;

	matrix::Vector<float, 3> thrust_sp;
	thrust_sp(0) = _control_sp(3);
	thrust_sp(1) = _control_sp(4);
	thrust_sp(2) = _control_sp(5);

	matrix::Matrix<float, 12, 8> &CI = _qp.GetCIRef();

	for (int i = 0; i < 4; ++i) {
		matrix::Matrix<float, 3, 1> cof = _motor_R[i].transpose() * thrust_sp;
		CI.slice<3, 1>(3 * i, i) = cof;
	}

	matrix::Vector<float, 6> &Ce0 = _qp.GetCe0Ref();
	Ce0 = -(_control_sp - _control_trim);

	if (_qp.Solve()) {
		_last_qp_success = true;
		_actuator_sp = _qp.GetOptimalVector();
		_last_success_actuator_sp = _actuator_sp;

	} else {
		_last_qp_success = false;
		_actuator_sp = _last_success_actuator_sp;
	}
}
