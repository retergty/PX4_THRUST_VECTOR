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
 * @file ActuatorEffectiveness2DRotors.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author ChenHuaMing <337367729@qq.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

class ActuatorEffectivenessTilts;

using namespace time_literals;

class ActuatorEffectiveness3DRotors : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class AxisConfiguration {
		Configurable, ///< axis can be configured
		FixedForward, ///< axis is fixed, pointing forwards (positive X)
		FixedUpwards, ///< axis is fixed, pointing upwards (negative Z)
	};

	static constexpr int NUM_ROTORS_MAX = 12;

	struct RotorGeometry {
		matrix::Vector3f position;
		matrix::Vector3f axis;
		matrix::Vector3f initial_axis;
		float thrust_coef;
		float moment_ratio;
		int tilt_index;
	};

	struct Geometry {
		RotorGeometry rotors[NUM_ROTORS_MAX];
		int num_rotors{0};
		bool propeller_torque_disabled{false};
		bool yaw_by_differential_thrust_disabled{false};
		bool propeller_torque_disabled_non_upwards{false}; ///< keeps propeller torque enabled for upward facing motors
		bool three_dimensional_thrust_disabled{false}; ///< for handling of tiltrotor VTOL, as they pass in 1D thrust and collective tilt
	};

	ActuatorEffectiveness3DRotors(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable);
	virtual ~ActuatorEffectiveness3DRotors() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	int computeEffectivenessMatrix(const Geometry &geometry,
				       EffectivenessMatrix &effectiveness, int actuator_start_index = 0);

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "3D Rotors"; }

	const Geometry &geometry() const { return _geometry; }

	matrix::Vector<float, 12> computeActuatorSetpoint(const matrix::Vector<float, 12> &force_vector);
private:
	constexpr static float SERVO_ANGLE_MAX = M_PI_F / 2.f;
	constexpr static float SERVO_ANGLE_MIN = -M_PI_F / 2.f;

	void updateParams() override;
	const AxisConfiguration _axis_config;

	matrix::Quaternionf _motor_q[4];

	matrix::Vector<float, 2> _servo_angle[4];
	float _rotor_speed[4];
	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t axis_x;
		param_t axis_y;
		param_t axis_z;
		param_t thrust_coef;
		param_t moment_ratio;
		param_t tilt_index;
	};
	ParamHandles _param_handles[NUM_ROTORS_MAX];

	Geometry _geometry{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count
	)
};
