#include "serial_test.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace serial_test
{
SerialTest *g_dev{nullptr};

static int start(const char *port,const int baudrate = 115200)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (port == nullptr) {
		PX4_ERR("no device specified");
		return -1;
	}

	/* create the driver */
	g_dev = new SerialTest(port,baudrate);

	if (g_dev == nullptr) {
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
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

} // namespace

extern "C" __EXPORT int serial_test_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int baudrate;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
			}
			break;
		case 'd':
			device_path = myoptarg;
			break;

		default:
			serial_test::usage();
			return -1;
		}
	}

	if (myoptind >= argc) {
		serial_test::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return serial_test::start(device_path,baudrate);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return serial_test::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return serial_test::status();
	}

	serial_test::usage();
	return -1;
}

