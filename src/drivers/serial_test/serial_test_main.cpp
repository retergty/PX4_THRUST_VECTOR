#include "serial_test.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);
int SerialTest::print_usage(const char *reason)
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

imu forward to serial, use baud rate 926100

### Examples

Attempt to start imu forward on a specified serial device.
$ imu_forward start -d /dev/ttyS1
Stop driver
$ imu_forward stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("serial test", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}
int SerialTest::print_status()
{
	print_info();
	return 0;
}
int SerialTest::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("serial_test", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
				   run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		_task_id = -1;
		return -errno;
	}
		_task_id = task_id;

	return 0;
}
SerialTest *SerialTest::instantiate(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int baudrate;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool error_flag = false;
	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;
		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if(error_flag)
	{
		return nullptr;
	}

	SerialTest * serial_test = nullptr;
	/* create the driver */

	serial_test = new SerialTest(device_path,baudrate);
	if (serial_test->init() != PX4_OK) {
		delete serial_test;
		serial_test = nullptr;
		return nullptr;
	}

	return serial_test;
}
int
SerialTest::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

extern "C" __EXPORT int serial_test_main(int argc, char *argv[])
{
	return SerialTest::main(argc, argv);
}

