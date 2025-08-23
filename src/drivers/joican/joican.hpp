#pragma once
#include "joican_raw_can_driver.hpp"
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_config.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module_params.h>
#include <mixer_module/mixer_module.hpp>
namespace joican
{

struct AbsPostionCtlFrame : public can_frame {
	static constexpr int CycleCount = 16384;  // +16384 = +360 deg , -16384 = -360 deg
	static constexpr float CountFactor = CycleCount / 2.f;
	static constexpr uint8_t CmdCode = 0xC2;
	static constexpr uint8_t DataLength = 0x5;
	AbsPostionCtlFrame(const uint32_t dev_id)
	{
		can_id = dev_id;
		can_dlc = DataLength;
		data[0] = CmdCode;
		// zero position
		memset(&data[1], 0, 4);
		return;
	};
	// [-1,1] <--> [-180,180]
	void setAbsPosition(const float abs_pos)
	{
		int pos_count = static_cast<int>(abs_pos * CountFactor);
		memcpy(&data[1], &pos_count, 4);
		return;
	};
};

class JoicanReceiver
{};

class Joican final : public ModuleBase<Joican>, public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = 8;
	Joican()
		: OutputModuleInterface(MODULE_NAME "-actuators-servo", px4::wq_configurations::rate_ctrl)
		, _can1_servo_posctl{ 0x1, 0x2, 0x3, 0x4 }
		, _can2_servo_posctl{ 0x1, 0x2, 0x3, 0x4 } {};
	~Joican() override {};

	int init();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;
	MixingOutput &mixingOutput()
	{
		return _mixing_output;
	}

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

private:
	void updateParams() override;

	void sendServoSetpoint();

	void setCountCorRet(const int16_t ret, int &sucess_count, int &fail_count);

private:
	MixingOutput _mixing_output{ "JOICAN_SV", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false };
	AbsPostionCtlFrame _can1_servo_posctl[4];
	AbsPostionCtlFrame _can2_servo_posctl[4];
	CanDriver _can1;
	CanDriver _can2;
	int16_t _can1_last_ret[4];
	int16_t _can2_last_ret[4];
	bool _is_init{ false };
	int _count{ 0 };
	int _can1_sucess_count{ 0 };
	int _can1_fail_count{ 0 };
	int _can2_sucess_count{ 0 };
	int _can2_fail_count{ 0 };

	uORB::SubscriptionInterval _parameter_update_sub{ ORB_ID(parameter_update), 1_s };
	perf_counter_t _cycle_perf{ perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle") };
	perf_counter_t _interval_perf{ perf_alloc(PC_INTERVAL, MODULE_NAME ": interval") };
};
}  // namespace joican
