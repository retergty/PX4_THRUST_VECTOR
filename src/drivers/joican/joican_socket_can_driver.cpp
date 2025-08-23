#include "joican_socket_can_driver.hpp"
#include <net/if.h>

#include <px4_platform_common/log.h>
#include "arm_internal.h"
#include "stm32.h"
namespace joican
{
int CanDriver::print_status()
{
	PX4_INFO("stm32h7 fdcan base address:%08x",(int)_priv->base);

	int CCCR = getreg32(_priv->base + STM32_FDCAN_CCCR_OFFSET);
	PX4_INFO("CCCR register: %08x",CCCR);

	int ECR = getreg32(_priv->base + STM32_FDCAN_ECR_OFFSET);
	const uint8_t cel = (uint8_t)(ECR >> FDCAN_ECR_CEL_SHIFT);
	PX4_INFO("ECR register: %08x, cel: %d",ECR,cel);

	int IER = getreg32(_priv->base + STM32_FDCAN_IE_OFFSET);
	PX4_INFO("IE register: %08x",IER);

	int TXBTO = getreg32(_priv->base + STM32_FDCAN_TXBTO_OFFSET);
	PX4_INFO("TXBTO register: %08x",TXBTO);

	int TXBAR = getreg32(_priv->base + STM32_FDCAN_TXBAR_OFFSET);
	PX4_INFO("TXBAR register: %08x",TXBAR);

	int TXBCR = getreg32(_priv->base + STM32_FDCAN_TXBCR_OFFSET);
	PX4_INFO("TXBCR register: %08x",TXBCR);

	int TXBCF = getreg32(_priv->base + STM32_FDCAN_TXBCF_OFFSET);
	PX4_INFO("TXBCF register: %08x",TXBCF);

	int TXFQS = getreg32(_priv->base + STM32_FDCAN_TXFQS_OFFSET);
	PX4_INFO("TXFQS register: %08x",TXFQS);

	int TXBC = getreg32(_priv->base + STM32_FDCAN_TXBC_OFFSET);
	PX4_INFO("TXBC register: %08x",TXBC);

	int NBTP = getreg32(_priv->base + STM32_FDCAN_NBTP_OFFSET);
	PX4_INFO("NBTP register: %08x",NBTP);

	int TXBTIE = getreg32(_priv->base + STM32_FDCAN_TXBTIE_OFFSET);
	PX4_INFO("TXBTIE register: %08x",TXBTIE);

	int SIDFC = getreg32(_priv->base + STM32_FDCAN_SIDFC_OFFSET);
	PX4_INFO("SIDFC register: %08x",SIDFC);

	int XIDFC = getreg32(_priv->base + STM32_FDCAN_XIDFC_OFFSET);
	PX4_INFO("XIDFC register: %08x",XIDFC);

	int RXESC = getreg32(_priv->base + STM32_FDCAN_RXESC_OFFSET);
	PX4_INFO("RXESC register: %08x",RXESC);

	int TXESC = getreg32(_priv->base + STM32_FDCAN_TXESC_OFFSET);
	PX4_INFO("TXESC register: %08x",TXESC);

	int RXF0C = getreg32(_priv->base + STM32_FDCAN_RXF0C_OFFSET);
	PX4_INFO("RXF0C register: %08x",RXF0C);

	int RXF1C = getreg32(_priv->base + STM32_FDCAN_RXF1C_OFFSET);
	PX4_INFO("RXF1C register: %08x",RXF1C);

	int RXBC = getreg32(_priv->base + STM32_FDCAN_RXBC_OFFSET);
	PX4_INFO("RXBC register: %08x",RXBC);

	int TXEFC = getreg32(_priv->base + STM32_FDCAN_TXEFC_OFFSET);
	PX4_INFO("TXEFC register: %08x",TXEFC);

	int TXEFS = getreg32(_priv->base + STM32_FDCAN_TXEFS_OFFSET);
	PX4_INFO("TXEFS register: %08x",TXEFS);

	int GFC = getreg32(_priv->base + STM32_FDCAN_GFC_OFFSET);
	PX4_INFO("GFC register: %08x",GFC);

	int PSR = getreg32(_priv->base + STM32_FDCAN_PSR_OFFSET);
	PX4_INFO("PSR register: %08x",PSR);
	return 0;
}
int CanDriver::socketInit(uint32_t index)
{
	sockaddr_can addr;
	ifreq ifr;

	_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	/* open socket */
	if (_fd < 0) {
		PX4_ERR("socket");
		return -1;
	}

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%li", index);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return -1;
	}

	// Setup TX msg
	_send_iov.iov_base = &_send_frame;
	_send_iov.iov_len = sizeof(struct can_frame);

	_send_msg.msg_iov    = &_send_iov;
	_send_msg.msg_iovlen = 1;
	_send_msg.msg_control =	 NULL;
	_send_msg.msg_controllen = 0;

	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;
	_recv_iov.iov_len = sizeof(struct can_frame);

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = NULL;
	_recv_msg.msg_controllen = 0;

	PX4_INFO("fd: %d",_fd);
	_s = sockfd_socket(_fd);
	struct can_conn_s * conn = (FAR struct can_conn_s *)_s->s_conn;
	_net_dev = conn->dev;
	_priv = (struct fdcan_driver_s *)_net_dev->d_private;
	PX4_INFO("socket: %p", _s);

	return 0;
}

int16_t CanDriver::send(const can_frame &frame)
{
	int res = -1;

	_send_frame.can_id = frame.can_id;
	_send_frame.can_dlc = frame.can_dlc;

	memcpy(_send_frame.data, frame.data, frame.can_dlc);

	res = sendmsg(_fd, &_send_msg, MSG_DONTWAIT);

	if (res > 0) {
		return 1;

	} else {
		return res;
	}

}

int16_t CanDriver::receive(can_frame &out_frame)
{
	int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result < 0) {
		return result;
	}

	out_frame.can_id = _recv_frame.can_id;

	if (_recv_frame.can_dlc > CAN_MAX_DLEN) {
		return -EFAULT;
	}

	out_frame.can_dlc = _recv_frame.can_dlc;
	memcpy(out_frame.data, _recv_frame.data, _recv_frame.can_dlc);

	return result;
}
}
