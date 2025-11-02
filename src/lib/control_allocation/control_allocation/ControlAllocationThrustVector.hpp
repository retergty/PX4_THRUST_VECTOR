#pragma once

#include "ControlAllocation.hpp"
#include <quad_prog/QuadProg.hpp>

// class QpManager
// {
// public:
// 	void updateQpState(const Matrix<Scalar, DIMISIONS, DIMISIONS> &G, const Vector<Scalar, DIMISIONS> &g0,
// 			   const Matrix<Scalar, DIMISIONS, EQST> &CE, const Vector<Scalar, EQST> &ce0,
// 			   const Matrix<Scalar, DIMISIONS, IEQST> &CI, const Vector<Scalar, IEQST> &ci0);
// private:
// 	QuadProg<float, 12, 6, 8> _takeoff_qp;
// 	QuadProg<float, 12, 6, 8> _optimal_qp;
// };

class ControlAllocationThrustVector: public ControlAllocation
{
public:
	static constexpr float FzFactor = 1.0f;
	static constexpr float QpLimits = 0.03f;
	static constexpr float QpLimitsTol = 0.02f;
	ControlAllocationThrustVector() = default;
	virtual ~ControlAllocationThrustVector() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;
	matrix::Vector<float, NUM_AXES> getAllocatedControl() const override
	{
		return _effectiveness * _qp.GetOptimalVector();
	}
	void print_status() override;

protected:
	/**
	 * update qp state if required.
	 *
	 */
	void updateQPState();

private:
	QuadProg<float, 12, 6, 8> _qp;
	matrix::Vector<float, NUM_ACTUATORS> _safe_actuator_sp;  	///< Actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _last_success_actuator_sp;
	matrix::Matrix<float, 3, 3> _motor_R[4];
	bool _last_qp_success{true};
	bool _qp_need_reinitialise{false};
};
