#pragma once
#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/can.h>
#include <netpacket/can.h>

namespace joican
{

class CanDriver
{
public:
	int socketInit(uint32_t index);

	// non-block send
	int16_t send(const can_frame &frame);

	// non-block receive
	int16_t receive(can_frame &out_frame);

	int getSocket() { return _fd; };

	int print_status();
private:
	//// Send msg structure
	struct iovec       _send_iov {};
	struct can_frame   _send_frame {};
	struct msghdr      _send_msg {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct can_frame   _recv_frame {};
	struct msghdr      _recv_msg {};

	int  _fd{-1};
	struct socket* _s{nullptr};
	struct net_driver_s *_net_dev{nullptr};
	struct fdcan_driver_s *_priv;
};
}
