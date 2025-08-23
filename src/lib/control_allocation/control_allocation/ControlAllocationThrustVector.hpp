#pragma once

#include "ControlAllocation.hpp"
#include <quad_prog/QuadProg.hpp>
class ControlAllocationThrustVector: public ControlAllocation
{
public:
	static constexpr float FzFactor = 5;
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
	bool _last_qp_success{true};
	bool _qp_need_reinitialise{false};
};
