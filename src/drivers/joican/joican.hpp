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
	static constexpr uint8_t DataLength = 0x08;
	static constexpr float Kp = 10.f;
	static constexpr float Kd = 1.f;

	static constexpr float PositionOffsetFactor = 12.5f;
	static constexpr float PositionMultiplyFactor = 65535.f / 25.f;

	static constexpr float VelocityOffsetFactor = 65.f;
	static constexpr float VelocityMultiplyFactor = 4095.f / 130.f;
	static constexpr uint16_t VelocityInt = static_cast<uint16_t>((0 + VelocityOffsetFactor) *
						VelocityMultiplyFactor); // 0 velocity feedword

	static constexpr float KpFactor = 4095.f / 500.f;
	static constexpr float KdFactor = 4095.f / 5.f;
	static constexpr uint16_t KpInt = static_cast<uint16_t>(Kp *KpFactor);
	static constexpr uint16_t KdInt = static_cast<uint16_t>(Kd *KdFactor);

	static constexpr float TorqueConstant = 0.919f;
	static constexpr float GearRatio = 8.f;
	static constexpr float TorqueOffsetFactor = 225.f * 0.919f / 8.f;
	static constexpr float TorqueMultiplyFactor = 4095.f / (2 *  TorqueOffsetFactor);
	static constexpr uint16_t TorqueInt = static_cast<uint16_t>((0 + TorqueOffsetFactor) *
					      TorqueMultiplyFactor); // 0 torque  feedword

	static constexpr float positionDecoder(const uint16_t pos_int)
	{
		return pos_int / PositionMultiplyFactor - PositionOffsetFactor;
	}
	static constexpr float velocityDecoder(const uint16_t vel_int)
	{
		return vel_int / VelocityMultiplyFactor - VelocityOffsetFactor;
	}
	static constexpr float torqueDecoder(const uint16_t tor_int)
	{
		return tor_int / TorqueMultiplyFactor - TorqueOffsetFactor;
	}

	static constexpr uint16_t positionEncoder(const float pos)
	{
		return static_cast<uint16_t>((pos + PositionOffsetFactor) * PositionMultiplyFactor);
	}
	static constexpr uint16_t velocityEncoder(const float vel)
	{
		return static_cast<uint16_t>((vel + VelocityOffsetFactor) * VelocityMultiplyFactor);
	}
	static constexpr uint16_t torqueEncoder(const float tor)
	{
		return static_cast<uint16_t>((tor + TorqueOffsetFactor) * TorqueMultiplyFactor);
	}

	AbsPostionCtlFrame(const uint32_t dev_id)
	{
		can_id = dev_id;
		can_dlc = DataLength;
		constexpr uint16_t pos_int = static_cast<uint16_t>((0 + PositionOffsetFactor) * PositionMultiplyFactor);
		data[0] = static_cast<uint8_t>(pos_int >> 8);
		data[1] = static_cast<uint8_t>(pos_int);
		data[2] = (VelocityInt >> 4);
		data[3] = ((VelocityInt & 0xF) << 4) | (KpInt >> 8);
		data[4] = static_cast<uint8_t>(KpInt);
		data[5] = KdInt >> 4;
		data[6] = ((KdInt & 0xF) << 4) | (TorqueInt >> 8);
		data[7] = static_cast<uint8_t>(TorqueInt);
		return;
	};
	void setAbsPosition(const float abs_pos)
	{
		uint16_t pos_int = static_cast<uint16_t>((abs_pos + PositionOffsetFactor) * PositionMultiplyFactor);
		data[0] = (pos_int >> 8);
		data[1] = pos_int;
		return;
	};
};
struct ServoEnableFrame : public can_frame {
	static constexpr uint8_t ServoEnableData[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	ServoEnableFrame(const uint32_t dev_id)
	{
		can_id = dev_id;
		can_dlc = sizeof(ServoEnableData);

		for (int i = 0; i < 8; ++i) {
			data[i]  = ServoEnableData[i];
		}
	}
};

struct ServoDisableFrame : public can_frame {
	static constexpr uint8_t ServoDisableData[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	ServoDisableFrame(const uint32_t dev_id)
	{
		can_id = dev_id;
		can_dlc = sizeof(ServoDisableData);

		for (int i = 0; i < 8; ++i) {
			data[i] = ServoDisableData[i];
		}
	}
};

struct ServoState {
	static constexpr uint8_t ERROR_FOCHIGH = 0x01;
	static constexpr uint8_t ERROR_OVERVOTAGE = 0x01 << 1;
	static constexpr uint8_t ERROR_LEAKVOTAGE = 0x01 << 2;
	static constexpr uint8_t ERROR_OVERTEMPERATURE = 0x01 << 3;
	static constexpr uint8_t ERROR_STARTFAIL = 0x01 << 4;
	static constexpr uint8_t ERROR_OVERCURRENT = 0x01 << 6;
	static constexpr uint8_t ERROR_SOFTWARE = 0x01 << 7;
	ServoState(const uint8_t can_id) : id(can_id) {};
	uint8_t id;
	float position{ 0.f };
	float velocity{ 0.f };
	float torque{ 0.f };
	float zero_offset{ 0.f };
	bool enable{0};
	uint8_t error_code;
};

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
	void ServoEnable();

	void ServoDisable();

	void updateParams() override;

	void handReceiveFrame(const can_frame &frame, const uint8_t can_instance);

	void sendServoSetpoint();

	void setCountCorRet(const int16_t ret, int &sucess_count, int &fail_count);

private:
	MixingOutput _mixing_output{ "JOICAN_SV", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false };

	AbsPostionCtlFrame _can1_servo_posctl[4];
	AbsPostionCtlFrame _can2_servo_posctl[4];

	CanDriver _can1;
	CanDriver _can2;

	int16_t _can1_last_ret[4] { 0, 0, 0, 0 };
	int16_t _can2_last_ret[4] { 0, 0, 0, 0 };

	bool _is_init{ false };
	int _count{ 0 };
	int _can1_sucess_count{ 0 };
	int _can1_fail_count{ 0 };
	int _can2_sucess_count{ 0 };
	int _can2_fail_count{ 0 };

	int _get_count{0};
	ServoState _can1_servo[4] {0x1, 0x2, 0x3, 0x4};
	ServoState _can2_servo[4] {0x1, 0x2, 0x3, 0x4};

	uORB::SubscriptionInterval _parameter_update_sub{ ORB_ID(parameter_update), 1_s };
	perf_counter_t _cycle_perf{ perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle") };
	perf_counter_t _interval_perf{ perf_alloc(PC_INTERVAL, MODULE_NAME ": interval") };
};
}  // namespace joican
