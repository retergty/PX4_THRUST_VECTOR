#pragma once

#include "ControlAllocation.hpp"
//#include <quad_prog/QuadProg.hpp>
#include <quad_prog/QuadProgNew.hpp>

#include <px4_platform_common/module_params.h>

// class QpManager
// {
// public:

// private:
// 	QuadProgNew<float, 12, 6, 8> _main_qp;
// 	QuadProgNew<float, 4, 3, 8> _diff_qp;
// };

class ControlAllocationThrustVector: public ControlAllocation
{
public:
	static constexpr float FzFactor = 0.30f;
	static constexpr float QpLimits = 0.20f;
	static constexpr float QpLimitsTol = 0.35f;
	static constexpr float DiffQpThrustTol = 0.2f;
	ControlAllocationThrustVector();
	virtual ~ControlAllocationThrustVector() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;
	matrix::Vector<float, NUM_AXES> getAllocatedControl() const override
	{
		return _effectiveness * _main_qp.GetOptimalVector();
	}
	void print_status() override;

	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t moment_ratio;
		param_t thrust_coef;
	};

	struct ThrustVectorUnitGeometry {
		matrix::Vector3f position; // thrust vector unit position in body frame
		matrix::SquareMatrix3f rotation; // body to thrust vector unit local frame
		float moment_ratio; // km
		float thrust_coef; //Ct
	};

	struct ThrustVectorUnitSetpoint {
		float InnerAngle{0};
		float OuterAngle{0};
		float Thrust{0.05};
	};

protected:
	/**
	 * update qp state if required.
	 *
	 */
	void updateQPState();
private:
	bool isThrustSetpointUpdated(const matrix::Vector<float, 6> control_sp) const;

	void computeMainQpSetpoint(const matrix::Vector<float, 12> &force_vector);
	matrix::Vector<float, 12> computeActuatorSetpoint() const;

	inline matrix::Dcmf calculateThrustVectorRotationMatrix(const int unit_index);

	inline void updateMainQpSolver();
private:
	ParamHandles _param_handles[4];

	QuadProgNew<float, 12, 6, 8> _main_qp;

	//DiagnoalMatrix<float, 12> _G;
	matrix::Vector<float, NUM_ACTUATORS> _safe_actuator_sp;  	///< Actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _last_success_actuator_sp;

	ThrustVectorUnitSetpoint _thrust_vector_unit_setpoint[4];
	ThrustVectorUnitGeometry _thrust_vector_unit_geometry[4];

	matrix::Vector<float, 6> _last_allocate_control_sp;

	bool _last_qp_success{false};
	bool _qp_need_reinitialise{false};
};
