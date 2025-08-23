/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on a serial/spi port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <poll.h>
#endif

#include <cstring>

#include <drivers/drv_sensor.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/Serial.hpp>

#include "serial_test.hpp"

static char test_buf[50];
//static char read_buf[100];
using namespace time_literals;

static void change_read_state(SerialTest::ReadState& state, const char get_ch, const char des_ch,
			      const SerialTest::ReadState right_to, const SerialTest::ReadState fail_to)
{
  if (get_ch == des_ch)
  {
    state = right_to;
  }
  else
  {
    state = fail_to;
  }
}

// 0: not yet
// 1: sucess
// -1: fail
int SerialTest::handleReceive(char* buf, int length)
{
  int read = 0;
  while (read < length)
  {
    char ch = buf[read++];
    switch (_read_state)
    {
      // sync 1
      case ReadState::NoSync:
	change_read_state(_read_state, ch, 0x11, ReadState::Sync1, ReadState::NoSync);
	break;
      case ReadState::Sync1:
	change_read_state(_read_state, ch, 0x22, ReadState::Sync2, ReadState::NoSync);
	break;
      case ReadState::Sync2:
	change_read_state(_read_state, ch, 0x33, ReadState::Sync3, ReadState::NoSync);
	break;
      case ReadState::Sync3:
	change_read_state(_read_state, ch, 0x44, ReadState::Sync4, ReadState::NoSync);
	break;
      case ReadState::Sync4:
	change_read_state(_read_state, ch, 0x44, ReadState::Value, ReadState::NoSync);
	break;
      case ReadState::Value:
	_last_read_value = ch;
	_read_state = ReadState::EndSync;
	break;
      case ReadState::EndSync:
	change_read_state(_read_state, ch, 0x55, ReadState::NoSync, ReadState::NoSync);
	break;
      default:
	break;
    }
  }
  return 0;
}

SerialTest::SerialTest(const char* port, const int baudrate)
  : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
  , _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": read"))
{
  /* store port name */
  strncpy(_port, port, sizeof(_port) - 1);

  /* enforce null termination */
  _port[sizeof(_port) - 1] = '\0';

  _baudrate = baudrate;

  ScheduleOnInterval(100_ms, 1_s);
}
SerialTest::~SerialTest()
{
  stop();
}
void SerialTest::stop()
{
  ScheduleClear();
}

void SerialTest::Run()
{
  perf_begin(_sample_perf);

  if (!_uart.isOpen())
  {
    // Configure UART port
    if (!_uart.setPort(_port))
    {
      PX4_ERR("Error configuring serial device on port %s", _port);
      return;
    }

    // Configure the desired baudrate if one was specified by the user.
    // Otherwise the default baudrate will be used.
    if (!_uart.setBaudrate(_baudrate))
    {
      PX4_ERR("Error setting baudrate to %u on %s", _baudrate, _port);
      return;
    }

    // Open the UART. If this is successful then the UART is ready to use.
    if (!_uart.open())
    {
      PX4_ERR("Error opening serial device  %s", _port);
      return;
    }
  }
  if (_uart.isOpen())
  {
    sprintf(test_buf, "serial test %d \n \r\n", _count++);
    _uart.write(&test_buf, 50);
  }

  perf_end(_sample_perf);
}
void SerialTest::print_info()
{
  PX4_INFO("Sending Serial Test\n");
  PX4_INFO("Now sending count %d\n", _count);
  PX4_INFO("Serial information:\n");
  PX4_INFO("configured serial port: %s, expected %s,", _uart.getPort(), _port);

  if (_uart.isOpen())
  {
    PX4_INFO("opening\n");
  }

  else
  {
    PX4_INFO("closed\n");
  }

  PX4_INFO("configured serial baudrate: %lu\n", _uart.getBaudrate());
  PX4_INFO("configured byte size: 8 bits, parity: none, step bits: 1 bit, flow control: no\n");
  perf_print_counter(_sample_perf);
}
int SerialTest::init()
{
  return PX4_OK;
}
