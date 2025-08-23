#pragma once
#include <nuttx/can.h>
#include "arm_internal.h"
#include "stm32.h"

namespace joican
{
#define CAN_MAX_DLEN 8
#define CONFIG_FDCAN1_BITRATE 1000000
#define CONFIG_FDCAN2_BITRATE 1000000

struct can_frame
{
  uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t __pad;   /* padding */
  uint8_t __res0;  /* reserved / padding */
  uint8_t __res1;  /* reserved / padding */
  uint8_t data[CAN_MAX_DLEN] aligned_data(8);
};

struct fdcan_driver_s;
// using can fifo
class CanDriver
{
public:
  int initCan(uint32_t index);

  int start();
  int stop();

  // non-block send
  int16_t send(const can_frame& frame);

  // is fifo full
  bool isFifoFull();

  // non-block receive
  int16_t receive(can_frame& out_frame);

  // wait untill buf not empty
  int waitReceive();

  int print_status();

private:
  //// Send msg structure
  struct can_frame _send_frame{};

  //// Receive msg structure
  struct can_frame _recv_frame{};

  struct fdcan_driver_s* _priv;
};
}  // namespace joican
