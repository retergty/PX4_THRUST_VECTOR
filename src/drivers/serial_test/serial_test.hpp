#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <poll.h>
#endif

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <cstdint>

class SerialTest : public px4::ScheduledWorkItem
{
public:
	static constexpr uint64_t VEHICLE_STATUS_SEND_INVERVAL_us = 2e5;
	SerialTest(const char *port,const int baudrate = 115200);
	~SerialTest() override;
	int init();
	void print_info();
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
private:
	void Run() override;
	void stop();
	int handleReceive(char * buf,int length);
	device::Serial  _uart {};
	char _port[20] {};
	int _baudrate;
	int _count{0};
	ReadState _read_state{ReadState::NoSync};
	int _read_error{0};
	perf_counter_t	_sample_perf;
	int _last_read_value{0};
};
