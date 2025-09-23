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
#include <termios.h>

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
// static char read_buf[100];
using namespace time_literals;

SerialTest::SerialTest(const char* port, const int baudrate)
  : _serial_receiver(*this), _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": read"))
{
  /* store port name */
  strncpy(_port, port, sizeof(_port) - 1);

  /* enforce null termination */
  _port[sizeof(_port) - 1] = '\0';

  _baudrate = baudrate;
}
SerialTest::~SerialTest()
{
  stop();
}
void SerialTest::stop()
{
  if (_uart_open)
  {
    ::close(_uart_fd);
    _uart_open = false;
  }

  if(_serial_receiver.isRunning())
  {
	_serial_receiver.stop();
  }
}
void SerialTest::run()
{

  while (!should_exit())
  {
    perf_begin(_sample_perf);

    if (!_uart_open)
    {
      _uart_fd = open_uart(_baudrate, _port);
      if (_uart_fd < 0)
      {
	PX4_ERR("Error opening serial device  %s", _port);
      }
      else
      {
	_uart_open = true;
	_serial_receiver.start();
      }
    }
    if (_uart_open)
    {
      sprintf(test_buf, "serial test %d \n \r\n", _count++);
      ::write(_uart_fd, &test_buf, 50);
    }

    px4_usleep(100000);	 // 100 ms
    perf_end(_sample_perf);
  }
}
void SerialTest::print_info()
{
  PX4_INFO("Sending Serial Test\n");
  PX4_INFO("Now sended count %d\n", _count);

  _serial_receiver.print_info();

  PX4_INFO("Serial information:\n");
  PX4_INFO("configured serial port: %s", _port);

  if (_uart_open)
  {
    PX4_INFO("opening\n");
  }

  else
  {
    PX4_INFO("closed\n");
  }

  PX4_INFO("configured serial baudrate: %u\n", _baudrate);
  PX4_INFO("configured byte size: 8 bits, parity: none, step bits: 1 bit, flow control: no\n");
  perf_print_counter(_sample_perf);
}
int SerialTest::init()
{
  return PX4_OK;
}

int SerialTest::open_uart(const int baud, const char* uart_name)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

  /* process baud rate */
  int speed;

  switch (baud)
  {
    case 0:
      speed = B0;
      break;

    case 50:
      speed = B50;
      break;

    case 75:
      speed = B75;
      break;

    case 110:
      speed = B110;
      break;

    case 134:
      speed = B134;
      break;

    case 150:
      speed = B150;
      break;

    case 200:
      speed = B200;
      break;

    case 300:
      speed = B300;
      break;

    case 600:
      speed = B600;
      break;

    case 1200:
      speed = B1200;
      break;

    case 1800:
      speed = B1800;
      break;

    case 2400:
      speed = B2400;
      break;

    case 4800:
      speed = B4800;
      break;

    case 9600:
      speed = B9600;
      break;

    case 19200:
      speed = B19200;
      break;

    case 38400:
      speed = B38400;
      break;

    case 57600:
      speed = B57600;
      break;

    case 115200:
      speed = B115200;
      break;

    case 230400:
      speed = B230400;
      break;

    case 460800:
      speed = B460800;
      break;

    case 500000:
      speed = B500000;
      break;

    case 921600:
      speed = B921600;
      break;

    case 1000000:
      speed = B1000000;
      break;

#ifdef B1500000

    case 1500000:
      speed = B1500000;
      break;
#endif

#ifdef B2000000

    case 2000000:
      speed = B2000000;
      break;
#endif

#ifdef B3000000

    case 3000000:
      speed = B3000000;
      break;
#endif

    default:
      PX4_ERR(
	  "Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, "
	  "57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
	  baud);
      return -EINVAL;
  }

  /* open uart */
  _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

  /*
   * Return here in the iridium mode since the iridium driver does not
   * support the subsequent function calls.
   */
  if (_uart_fd < 0)
  {
    return _uart_fd;
  }

  /* Try to set baud rate */
  struct termios uart_config;
  int termios_state;

  /* Initialize the uart config */
  if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0)
  {
    PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
    ::close(_uart_fd);
    return -1;
  }

  /* Clear ONLCR flag (which appends a CR for every LF) */
  uart_config.c_oflag &= ~ONLCR;

  /* Set baud rate */
  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
  {
    PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
    ::close(_uart_fd);
    return -1;
  }

  if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0)
  {
    PX4_WARN("ERR SET CONF %s\n", uart_name);
    ::close(_uart_fd);
    return -1;
  }

  return _uart_fd;
}
