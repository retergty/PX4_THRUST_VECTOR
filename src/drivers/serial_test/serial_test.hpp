#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <poll.h>
#endif

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <cstdint>

class SerialTest;
class SerialTestReceiver
{
public:
	enum class ReadState
	{
		NoSync,
		Sync1,
		Sync2,
		Sync3,
		Sync4,
		Value,
		EndSync,
	};
	SerialTestReceiver(SerialTest& serial_test) : _serial_test(serial_test) {};
	int handleReceive(uint8_t * buf,int length);
	static void * start_trampoline(void *context);
	void print_info();
	void run();
	void start();
	void stop();
	bool isRunning() { return _receiver_start;};

	void changeReadState(const uint8_t get_ch, const uint8_t des_ch, const SerialTestReceiver::ReadState right_to, const SerialTestReceiver::ReadState fail_to);
private:
	SerialTest& _serial_test;
	ReadState _read_state{ReadState::NoSync};
	int _last_read_value{-1};
	int _read_error{0};
	int _read_success{0};
	pthread_t _thread {};
	bool _receiver_start{false};
};

class SerialTest : public ModuleBase<SerialTest>, public ModuleParams
{
public:
	static constexpr uint64_t VEHICLE_STATUS_SEND_INVERVAL_us = 2e5;
	SerialTest(const char *port,const int baudrate = 115200);
	~SerialTest() override;
	int init();
	void run() override;
	void print_info();
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);
	/** @see ModuleBase */
	static SerialTest *instantiate(int argc, char *argv[]);
	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;

	int open_uart(const int baud, const char *uart_name);

	int get_uart_fd() {return _uart_fd;};

	bool should_exit() { return ModuleBase::should_exit();};
private:
	SerialTestReceiver _serial_receiver;
	void stop();
	//device::Serial  _uart {};
	int _uart_fd{-1};
	bool _uart_open{false};

	char _port[20] {};
	int _baudrate;
	int _count{0};

	perf_counter_t	_sample_perf;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SER_TET_INV>) _param_serial_invert
	)
};
