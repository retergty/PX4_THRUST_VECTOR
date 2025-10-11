#include "joican_raw_can_driver.hpp"

#include "arm_internal.h"
#include "stm32.h"
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <px4_platform_common/log.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include "ring_buffer.hpp"

#define JOICAN_DEBUG 0

namespace joican
{
/* Message RAM Configuration ************************************************/

#define WORD_LENGTH 4U

#define FIFO_ELEMENT_SIZE 4 /* size (in Words) of a FIFO element in message RAM (CAN_MTU / 4) */
#define NUM_RX_FIFO0 64	    /* 64 elements max for RX FIFO0 */
#define NUM_RX_FIFO1 0	    /* No elements for RX FIFO1 */
#define NUM_TX_FIFO 32	    /* 32 elements max for TX FIFO */
/* Special address description flags for the CAN_ID */

#define CAN_EFF_FLAG 0x80000000 /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000 /* Remote transmission request */
#define CAN_ERR_FLAG 0x20000000 /* Error message frame */

/* Valid bits in CAN ID for frame formats */

#define CAN_SFF_MASK 0x000007ff /* Standard frame format (SFF) */
#define CAN_EFF_MASK 0x1fffffff /* Extended frame format (EFF) */
#define CAN_ERR_MASK 0x1fffffff /* Omit EFF, RTR, ERR flags */

/* CAN Clock Configuration **************************************************/

#define STM32_FDCANCLK STM32_HSE_FREQUENCY
#define CLK_FREQ STM32_FDCANCLK
#define PRESDIV_MAX 256

/* Interrupts ***************************************************************/

#define FDCAN_IR_MASK 0x3fcfffff /* Mask of all non-reserved bits in FDCAN_IR */

/* CAN ID word, as defined by FDCAN device (Note xtd/rtr/esi bit positions) */

union can_id_u {
	volatile uint32_t can_id;
	struct {
		volatile uint32_t extid : 29;
		volatile uint32_t resex : 3;
	};
	struct {
		volatile uint32_t res : 18;
		volatile uint32_t stdid : 11;
		volatile uint32_t rtr : 1;
		volatile uint32_t xtd : 1;
		volatile uint32_t esi : 1;
	};
};

/* Union of 4 bytes as 1 register */

union payload_u {
	volatile uint32_t word;
	struct {
		volatile uint32_t b00 : 8;
		volatile uint32_t b01 : 8;
		volatile uint32_t b02 : 8;
		volatile uint32_t b03 : 8;
	};
};

/* Message RAM Structures ***************************************************/

/* Rx FIFO Element Header -- RM0433 pg 2536 */

union rx_fifo_header_u {
	struct {
		volatile uint32_t w0;
		volatile uint32_t w1;
	};

	struct {
		/* First word */
		union can_id_u id;

		/* Second word */
		volatile uint32_t rxts : 16; /* Rx timestamp */
		volatile uint32_t dlc : 4;	 /* Data length code */
		volatile uint32_t brs : 1;	 /* Bitrate switching */
		volatile uint32_t fdf : 1;	 /* FD frame */
		volatile uint32_t res : 2;	 /* Reserved for Tx Event */
		volatile uint32_t fidx : 7;	 /* Filter index */
		volatile uint32_t anmf : 1;	 /* Accepted non-matching frame */
	};
};

/* Tx FIFO Element Header -- RM0433 pg 2538 */

union tx_fifo_header_u {
	struct {
		volatile uint32_t w0;
		volatile uint32_t w1;
	};

	struct {
		/* First word */

		union can_id_u id;

		/* Second word */

		volatile uint32_t res1 : 16; /* Reserved for Tx Event timestamp */
		volatile uint32_t dlc : 4;	 /* Data length code */
		volatile uint32_t brs : 1;	 /* Bitrate switching */
		volatile uint32_t fdf : 1;	 /* FD frame */
		volatile uint32_t res2 : 1;	 /* Reserved for Tx Event */
		volatile uint32_t efc : 1;	 /* Event FIFO control */
		volatile uint32_t mm : 8;	 /* Message marker (user data; copied to Tx Event) */
	};
};

/* Rx FIFO Element */

struct rx_fifo_s {
	union rx_fifo_header_u header;
	union payload_u data[2]; /* 8-byte Classic payload */
};

/* Tx FIFO Element */

struct tx_fifo_s {
	union tx_fifo_header_u header;
	union payload_u data[2]; /* 8-byte Classic payload */
};

/* Tx Mailbox Status Tracking */

#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats {
	int8_t pending;
};

/* FDCAN Device hardware configuration **************************************/

struct fdcan_config_s {
	uint32_t tx_pin;    /* GPIO configuration for TX */
	uint32_t rx_pin;    /* GPIO configuration for RX */
	uint32_t mb_irq[2]; /* FDCAN Interrupt 0, 1 (Rx, Tx) */
};

struct fdcan_bitseg {
	uint32_t bitrate;
	uint8_t sjw;
	uint8_t bs1;
	uint8_t bs2;
	uint8_t prescaler;
};

struct fdcan_message_ram {
	uint32_t filt_stdid_addr;
	uint32_t filt_extid_addr;
	uint32_t rxfifo0_addr;
	uint32_t rxfifo1_addr;
	uint32_t txfifo_addr;
	uint8_t n_stdfilt;
	uint8_t n_extfilt;
	uint8_t n_rxfifo0;
	uint8_t n_rxfifo1;
	uint8_t n_txfifo;
};

static const struct fdcan_config_s stm32_fdcan0_config = {
	.tx_pin      = GPIO_CAN1_TX,
	.rx_pin      = GPIO_CAN1_RX,
	.mb_irq      =
	{
		STM32_IRQ_FDCAN1_0,
		STM32_IRQ_FDCAN1_1,
	},
};

static const struct fdcan_config_s stm32_fdcan1_config = {
	.tx_pin      = GPIO_CAN2_TX,
	.rx_pin      = GPIO_CAN2_RX,
	.mb_irq      =
	{
		STM32_IRQ_FDCAN2_0,
		STM32_IRQ_FDCAN2_1,
	},
};

struct fdcan_driver_s {
	const struct fdcan_config_s *config; /* Pin config */
	uint8_t iface_idx;		       /* FDCAN interface index (0 or 1) */
	uint32_t base;		       /* FDCAN base address */

	struct fdcan_bitseg arbi_timing; /* Timing for arbitration phase */

	struct fdcan_message_ram message_ram; /* Start addresses for each reagion of Message RAM */
	struct rx_fifo_s *rx;			/* Pointer to Rx FIFO0 in Message RAM */
	struct tx_fifo_s *tx;			/* Pointer to Tx mailboxes in Message RAM */

	struct txmbstats txmb[NUM_TX_FIFO]; /* Track deadline and status of every Tx entry */

	sem_t receive_sem;

	RingBuffer<can_frame, NUM_RX_FIFO0> rx_buf;

#if JOICAN_DEBUG==1
	int _re_count {0};
	int _re_lost{0};
	int _re_error{0};

	int _send_count{0};
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fdcan_driver_s g_fdcan0;

static struct fdcan_driver_s g_fdcan1;

static bool g_apb1h_init = false;

/* Helper functions */
static void fdcan_apb1hreset(void);
static void fdcan_setinit(uint32_t base, uint32_t init);
static void fdcan_setenable(uint32_t base, uint32_t enable);
static void fdcan_setconfig(uint32_t base, uint32_t config_enable);
static bool fdcan_waitccr_change(uint32_t base, uint32_t mask, uint32_t target_state);

static void fdcan_enable_interrupts(struct fdcan_driver_s *priv);
static void fdcan_disable_interrupts(struct fdcan_driver_s *priv);

static int32_t fdcan_bittiming(struct fdcan_bitseg *timing);

/* Interrupt handling */
static void fdcan_receive(struct fdcan_driver_s *priv);
static void fdcan_txdone(struct fdcan_driver_s *priv);

static int fdcan_interrupt(int irq, void *context, void *arg);

static void fdcan_reset(struct fdcan_driver_s *priv);
/****************************************************************************
 * Function: fdcan_apb1hreset
 *
 * Description:
 *   Reset the periheral bus clock used by FDCAN
 *   Note that this will reset all configuration of all FDCAN peripherals
 *
 ****************************************************************************/

static void fdcan_apb1hreset(void)
{
	/* Reset the FDCAN's peripheral bus clock */
	modifyreg32(STM32_RCC_APB1HENR, 0, RCC_APB1HENR_FDCANEN);
	modifyreg32(STM32_RCC_APB1HRSTR, 0, RCC_APB1HRSTR_FDCANRST);
	modifyreg32(STM32_RCC_APB1HRSTR, RCC_APB1HRSTR_FDCANRST, 0);
}

/****************************************************************************
 * Function: fdcan_setinit
 *
 * Description:
 *   Enter / Exit initialization mode
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Enter init mode; false: Exit init mode
 *
 ****************************************************************************/

static void fdcan_setinit(uint32_t base, uint32_t init)
{
	if (init) {
		/* Enter hardware initialization mode */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);
		fdcan_waitccr_change(base, FDCAN_CCCR_INIT, FDCAN_CCCR_INIT);

	} else {
		/* Exit hardware initialization mode */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);
		fdcan_waitccr_change(base, FDCAN_CCCR_INIT, 0);
	}
}

/****************************************************************************
 * Function: fdcan_setenable
 *
 * Description:
 *   Power On / Power Off the device with the Clock Stop Request bit
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Power on the device; false: Power off the device
 *
 ****************************************************************************/

static void fdcan_setenable(uint32_t base, uint32_t enable)
{
	if (enable) {
		/* Clear CSR bit */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CSR, 0);
		fdcan_waitccr_change(base, FDCAN_CCCR_CSA, 0);

	} else {
		/* Set CSR bit */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CSR);
		fdcan_waitccr_change(base, FDCAN_CCCR_CSA, 1);
	}
}

/****************************************************************************
 * Function: fdcan_setconfig
 *
 * Description:
 *   Enter / Exit Configuration Changes Enabled mode
 *
 * Input Parameters:
 *   base - The base pointer of the FDCAN peripheral
 *   init - true: Enter config mode; false: Exit config mode
 *
 ****************************************************************************/

static void fdcan_setconfig(uint32_t base, uint32_t config_enable)
{
	if (config_enable) {
		/* Configuration Changes Enabled (CCE) mode */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);
		fdcan_waitccr_change(base, FDCAN_CCCR_CCE, 1);

	} else {
		/* Exit CCE mode */

		modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);
		fdcan_waitccr_change(base, FDCAN_CCCR_CCE, 0);
	}
}

/****************************************************************************
 * Function: fdcan_waitccr_change
 *
 * Description:
 *   Wait for the CCR register to accept a requested change.
 *   Timeout after ~10ms.
 *
 * Input Parameters:
 *   base         - The base pointer of the FDCAN peripheral
 *   mask         - Mask to apply to the CCR register value
 *   target_state - Target masked value to wait for
 *
 * Returned Value:
 *   true on success; false on timeout
 *
 ****************************************************************************/

static bool fdcan_waitccr_change(uint32_t base, uint32_t mask, uint32_t target_state)
{
	const unsigned timeout = 1000;

	for (unsigned wait_ack = 0; wait_ack < timeout; wait_ack++) {
		const bool state = (getreg32(base + STM32_FDCAN_CCCR_OFFSET) & mask);

		if (state == target_state) {
			return true;
		}

		up_udelay(10);
	}

	return false;
}

/****************************************************************************
 * Function: fdcan_enable_interrupts
 *
 * Description:
 *   Enable all interrupts used by this driver
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Assumptions:
 *   The peripheral is in Configuration Changes Enabled (CCE) mode
 *
 ****************************************************************************/

static void fdcan_enable_interrupts(struct fdcan_driver_s *priv)
{
	/* Enable both interrupt lines at the device level */

	const uint32_t ile = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;
	modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, 0, ile);

	/* Enable both lines at the NVIC level */

	up_enable_irq(priv->config->mb_irq[0]);
	up_enable_irq(priv->config->mb_irq[1]);
}

/****************************************************************************
 * Function: fdcan_disable_interrupts
 *
 * Description:
 *   Disable all interrupts used by this driver
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Assumptions:
 *   The peripheral is in Configuration Changes Enabled (CCE) mode
 *
 ****************************************************************************/

static void fdcan_disable_interrupts(struct fdcan_driver_s *priv)
{
	/* Disable both lines at the NVIC level */

	up_disable_irq(priv->config->mb_irq[0]);
	up_disable_irq(priv->config->mb_irq[1]);

	/* Disable both interrupt lines at the device level */

	const uint32_t ile = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;
	modifyreg32(priv->base + STM32_FDCAN_ILE_OFFSET, ile, 0);
}

/****************************************************************************
 * Name: fdcan_bittiming
 *
 * Description:
 *   Convert desired bitrate to FDCAN bit segment values
 *   The computed values apply to both data and arbitration phases
 *
 * Input Parameters:
 *   timing - structure to store bit timing
 *
 * Returned Value:
 *   OK on success; >0 on failure.
 ****************************************************************************/

int32_t fdcan_bittiming(struct fdcan_bitseg *timing)
{
	/* Implementation ported from PX4's uavcan_drivers/stm32[h7]
	 *
	 * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe
	 *  MicroControl GmbH & Co. KG
	 *  CAN in Automation, 2003
	 *
	 * According to the source, optimal quanta per bit are:
	 *   Bitrate        Optimal Maximum
	 *   1000 kbps      8       10
	 *   500  kbps      16      17
	 *   250  kbps      16      17
	 *   125  kbps      16      17
	 */

	const uint32_t target_bitrate = timing->bitrate;
	static const int32_t max_bs1 = 16;
	static const int32_t max_bs2 = 8;
	const uint8_t max_quanta_per_bit = (timing->bitrate >= 1000000) ? 10 : 17;
	static const int max_sp_location = 900;

	/* Computing (prescaler * BS):
	 *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))
	 *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
	 * let:
	 *   BS = 1 + BS1 + BS2
	 *     (BS == total number of time quanta per bit)
	 *   PRESCALER_BS = PRESCALER * BS
	 * ==>
	 *   PRESCALER_BS = PCLK / BITRATE
	 */

	const uint32_t prescaler_bs = CLK_FREQ / target_bitrate;

	/* Find prescaler value such that the number of quanta per bit is highest */

	uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

	while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
		if (bs1_bs2_sum <= 2) {
			PX4_ERR("Target bitrate too high - no solution possible.");
			return 1; /* No solution */
		}

		bs1_bs2_sum--;
	}

	const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

	if ((prescaler < 1U) || (prescaler > 1024U)) {
		PX4_ERR("Target bitrate invalid - bad prescaler.");
		return 2; /* No solution */
	}

	/* Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
	 * We need to find the values so that the sample point is as close as
	 * possible to the optimal value.
	 *
	 *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]
	 *     (Where 7/8 is 0.875, the recommended sample point location)
	 *   {{bs2 -> (1 + bs1)/7}}
	 *
	 * Hence:
	 *   bs2 = (1 + bs1) / 7
	 *   bs1 = (7 * bs1_bs2_sum - 1) / 8
	 *
	 * Sample point location can be computed as follows:
	 *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
	 *
	 * Since the optimal solution is so close to the maximum, we prepare two
	 * solutions, and then pick the best one:
	 *   - With rounding to nearest
	 *   - With rounding to zero
	 */

	/* First attempt with rounding to nearest */

	uint8_t bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) + 4) / 8;
	uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
	uint16_t sample_point_permill = (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));

	if (sample_point_permill > max_sp_location) {
		/* Second attempt with rounding to zero */

		bs1 = (7 * bs1_bs2_sum - 1) / 8;
		bs2 = bs1_bs2_sum - bs1;
	}

	bool valid = (bs1 >= 1) && (bs1 <= max_bs1) && (bs2 >= 1) && (bs2 <= max_bs2);

	/* Final validation
	 * Helpful Python:
	 * def sample_point_from_btr(x):
	 *     assert 0b0011110010000000111111000000000 & x == 0
	 *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
	 *     return (1+ts1+1)/(1+ts1+1+ts2+1)
	 */

	if (target_bitrate != (CLK_FREQ / (prescaler * (1 + bs1 + bs2))) || !valid) {
		PX4_ERR("Target bitrate invalid - solution does not match.");
		return 3; /* Solution not found */
	}

	timing->bs1 = (uint8_t)(bs1 - 1);
	timing->bs2 = (uint8_t)(bs2 - 1);
	timing->prescaler = (uint16_t)(prescaler - 1);
	timing->sjw = 0; /* Which means one */

	return 0;
}

/****************************************************************************
 * Function: fdcan_interrupt
 *
 * Description:
 *   Common handler for all enabled FDCAN interrupts
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - Unused
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int fdcan_interrupt(int irq, void *context, void *arg)
{
	switch (irq) {
	case STM32_IRQ_FDCAN1_0:
		fdcan_receive(&g_fdcan0);
		break;

	case STM32_IRQ_FDCAN1_1:
		fdcan_txdone(&g_fdcan0);
		break;

	case STM32_IRQ_FDCAN2_0:
		fdcan_receive(&g_fdcan1);
		break;

	case STM32_IRQ_FDCAN2_1:
		fdcan_txdone(&g_fdcan1);
		break;

	default:
		PX4_ERR("Unexpected IRQ [%d]\n", irq);
		return -EINVAL;
	}

	return OK;
}
static void fdcan_receive(struct fdcan_driver_s *priv)
{
	/* Check the interrupt value to determine which FIFO to read */
	uint32_t irflags = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

	const uint32_t ir_fifo0 = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
	const uint32_t ir_fifo1 = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
	uint8_t fifo_id;

	if (irflags & ir_fifo0) {
		irflags = ir_fifo0;
		fifo_id = 0;

	} else if (irflags & ir_fifo1) {
		irflags = ir_fifo1;
		fifo_id = 1;

	} else {
		//"ERROR: Bad RX IR flags";
#if JOICAN_DEBUG==1
		priv->_re_error++;
#endif
		return;
	}

	/* Bitwise register definitions are the same for FIFO 0/1
	 *
	 *   FDCAN_RXFnC_F0S:   Rx FIFO Size
	 *   FDCAN_RXFnS_RF0L:  Rx Message Lost
	 *   FDCAN_RXFnS_F0FL:  Rx FIFO Fill Level
	 *   FDCAN_RXFnS_F0GI:  Rx FIFO Get Index
	 *
	 * So we will use only the RX FIFO0 register definitions for simplicity
	 */

	uint32_t offset_rxfnc = (fifo_id == 0) ? STM32_FDCAN_RXF0C_OFFSET : STM32_FDCAN_RXF1C_OFFSET;
	uint32_t offset_rxfns = (fifo_id == 0) ? STM32_FDCAN_RXF0S_OFFSET : STM32_FDCAN_RXF1S_OFFSET;
	uint32_t offset_rxfna = (fifo_id == 0) ? STM32_FDCAN_RXF0A_OFFSET : STM32_FDCAN_RXF1A_OFFSET;

	volatile uint32_t *const rxfnc = (uint32_t *)(priv->base + offset_rxfnc);
	volatile uint32_t *const rxfns = (uint32_t *)(priv->base + offset_rxfns);
	volatile uint32_t *const rxfna = (uint32_t *)(priv->base + offset_rxfna);

	/* Check number of elements in message RAM allocated to this FIFO */

	if ((*rxfnc & FDCAN_RXF0C_F0S) == 0) {
		//("ERROR: No RX FIFO elements allocated");
#if JOICAN_DEBUG==1
		priv->_re_error++;
#endif
		return;
	}

	/* Check for message lost; count an error */

	if ((*rxfns & FDCAN_RXF0S_RF0L) != 0) {
#if JOICAN_DEBUG==1
		priv->_re_lost++;
#endif
		return;
	}

	/* Check number of elements available (fill level) */

	const uint8_t n_elem = (*rxfns & FDCAN_RXF0S_F0FL);

	if (n_elem == 0) {
		//("RX interrupt but 0 frames available");
#if JOICAN_DEBUG==1
		priv->_re_error++;
#endif
		return;
	}

	struct rx_fifo_s *rf = NULL;

	while ((*rxfns & FDCAN_RXF0S_F0FL) > 0) {
		/* Copy the frame from message RAM */
#if JOICAN_DEBUG==1
		priv->_re_count++;
#endif
		const uint8_t index = (*rxfns & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_SHIFT;

		rf = &priv->rx[index];
		struct can_frame frame;

		if (rf->header.id.xtd) {
			frame.can_id = CAN_EFF_MASK & rf->header.id.extid;
			frame.can_id |= CAN_EFF_FLAG;

		} else {
			frame.can_id = CAN_SFF_MASK & rf->header.id.stdid;
		}

		if (rf->header.id.rtr) {
			frame.can_id |= CAN_RTR_FLAG;
		}

		frame.can_dlc = rf->header.dlc;

		memcpy(frame.data, rf->data, frame.can_dlc);

		/* Acknowledge receipt of this FIFO element */

		putreg32(index, rxfna);

		priv->rx_buf.push(frame);
	}

	//sem_post(&priv->receive_sem);
	/* Write the corresponding interrupt bits to reset these interrupts */
	putreg32(irflags, priv->base + STM32_FDCAN_IR_OFFSET);
}
/****************************************************************************
 * Function: fdcan_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from interrupt context
 *
 ****************************************************************************/

static void fdcan_txdone(struct fdcan_driver_s *priv)
{
	/* Read and reset the interrupt flag */
	uint32_t ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

	if (ir & FDCAN_IR_TC) {
		putreg32(FDCAN_IR_TC, priv->base + STM32_FDCAN_IR_OFFSET);

	} else {
		//("Unexpected FCAN interrupt on line 1\n");
		return;
	}

	for (uint8_t i = 0; i < NUM_TX_FIFO; i++) {
		if ((getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET) & (1 << i)) > 0) {
			/* Transmission Occurred in buffer i
			 *   (Not necessarily a 'new' transmission, however)
			 * Check that it's a new transmission, not a previously handled
			 * transmission
			 */

			struct txmbstats *txi = &priv->txmb[i];

			if (txi->pending == TX_BUSY) {
				/* This is a transmission that just now completed */
				txi->pending = TX_FREE;
			}
		}
	}
}

/****************************************************************************
 * Function: fdcan_reset
 *
 * Description:
 *   Put the device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device has previously been initialized, including message RAM
 ****************************************************************************/

static void fdcan_reset(struct fdcan_driver_s *priv)
{
	/* Request Init Mode */
	irqstate_t flags = enter_critical_section();

	fdcan_setenable(priv->base, 1);
	fdcan_setinit(priv->base, 1);

	/* Enable Configuration Change Mode */

	fdcan_setconfig(priv->base, 1);

	/* Disable interrupts and clear all interrupt flags */

	fdcan_disable_interrupts(priv);

	putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

	/* Clear all message RAM mailboxes if initialized */
	const uint8_t n_data_words = 2;

	if (priv->rx) {
		for (uint32_t i = 0; i < NUM_RX_FIFO0; i++) {
			priv->rx[i].header.w1 = 0x0;
			priv->rx[i].header.w0 = 0x0;

			for (uint8_t j = 0; j < n_data_words; j++) {
				priv->rx[i].data[j].word = 0x0;
			}
		}
	}

	if (priv->tx) {
		for (uint32_t i = 0; i < NUM_TX_FIFO; i++) {
			priv->tx[i].header.w1 = 0x0;
			priv->tx[i].header.w0 = 0x0;

			for (uint8_t j = 0; j < n_data_words; j++) {
				priv->tx[i].data[j].word = 0x0;
			}
		}
	}

	/* Power off the device -- See RM0433 pg 2493 */

	fdcan_setinit(priv->base, 0);
	fdcan_setenable(priv->base, 0);

	leave_critical_section(flags);
}
#if JOICAN_DEBUG==1
static void CanRegisterDump(const struct fdcan_driver_s *const priv)
{
	PX4_INFO("stm32h7 fdcan base address:%08x", (int)priv->base);

	int CCCR = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
	PX4_INFO("CCCR register: %08x", CCCR);

	int ECR = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
	const uint8_t cel = (uint8_t)(ECR >> FDCAN_ECR_CEL_SHIFT);
	PX4_INFO("ECR register: %08x, cel: %d", ECR, cel);

	int IER = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
	PX4_INFO("IE register: %08x", IER);

	int TXBTO = getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET);
	PX4_INFO("TXBTO register: %08x", TXBTO);

	int TXBAR = getreg32(priv->base + STM32_FDCAN_TXBAR_OFFSET);
	PX4_INFO("TXBAR register: %08x", TXBAR);

	int TXBCR = getreg32(priv->base + STM32_FDCAN_TXBCR_OFFSET);
	PX4_INFO("TXBCR register: %08x", TXBCR);

	int TXBCF = getreg32(priv->base + STM32_FDCAN_TXBCF_OFFSET);
	PX4_INFO("TXBCF register: %08x", TXBCF);

	int TXFQS = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
	PX4_INFO("TXFQS register: %08x", TXFQS);

	int TXBC = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
	PX4_INFO("TXBC register: %08x", TXBC);

	int NBTP = getreg32(priv->base + STM32_FDCAN_NBTP_OFFSET);
	PX4_INFO("NBTP register: %08x", NBTP);

	int TXBTIE = getreg32(priv->base + STM32_FDCAN_TXBTIE_OFFSET);
	PX4_INFO("TXBTIE register: %08x", TXBTIE);

	int SIDFC = getreg32(priv->base + STM32_FDCAN_SIDFC_OFFSET);
	PX4_INFO("SIDFC register: %08x", SIDFC);

	int XIDFC = getreg32(priv->base + STM32_FDCAN_XIDFC_OFFSET);
	PX4_INFO("XIDFC register: %08x", XIDFC);

	int RXESC = getreg32(priv->base + STM32_FDCAN_RXESC_OFFSET);
	PX4_INFO("RXESC register: %08x", RXESC);

	int TXESC = getreg32(priv->base + STM32_FDCAN_TXESC_OFFSET);
	PX4_INFO("TXESC register: %08x", TXESC);

	int RXF0C = getreg32(priv->base + STM32_FDCAN_RXF0C_OFFSET);
	PX4_INFO("RXF0C register: %08x", RXF0C);

	int RXF1C = getreg32(priv->base + STM32_FDCAN_RXF1C_OFFSET);
	PX4_INFO("RXF1C register: %08x", RXF1C);

	int RXBC = getreg32(priv->base + STM32_FDCAN_RXBC_OFFSET);
	PX4_INFO("RXBC register: %08x", RXBC);

	int TXEFC = getreg32(priv->base + STM32_FDCAN_TXEFC_OFFSET);
	PX4_INFO("TXEFC register: %08x", TXEFC);

	int TXEFS = getreg32(priv->base + STM32_FDCAN_TXEFS_OFFSET);
	PX4_INFO("TXEFS register: %08x", TXEFS);

	int GFC = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
	PX4_INFO("GFC register: %08x", GFC);

	int PSR = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);
	PX4_INFO("PSR register: %08x", PSR);

	int TEST = getreg32(priv->base + STM32_FDCAN_TEST_OFFSET);
	PX4_INFO("TEST register: %08x", TEST);


	int RXF0S = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
	PX4_INFO("RXF0S register: %08x", RXF0S);
}
#endif
int CanDriver::print_status()
{
#if JOICAN_DEBUG==1
	CanRegisterDump(_priv);
	PX4_INFO("received frame in candriver %d", _priv->_re_count);
	PX4_INFO("received frame lost count %d", _priv->_re_lost);
	PX4_INFO("received frame error count %d", _priv->_re_error);
	PX4_INFO("send frame count %d", _priv->_send_count);
#endif
	return 0;
}
int CanDriver::initCan(uint32_t index)
{
	if (index == 0) {
		_priv = &g_fdcan0;
		_priv->base = STM32_FDCAN1_BASE;
		_priv->iface_idx = 0;
		_priv->config = &stm32_fdcan0_config;
		_priv->arbi_timing.bitrate = CONFIG_FDCAN1_BITRATE;
		sem_init(&_priv->receive_sem, 0, 0);

	} else if (index == 1) {
		_priv = &g_fdcan1;
		_priv->base = STM32_FDCAN2_BASE;
		_priv->iface_idx = 1;
		_priv->config = &stm32_fdcan1_config;
		_priv->arbi_timing.bitrate = CONFIG_FDCAN2_BITRATE;
		sem_init(&_priv->receive_sem, 0, 0);

	} else {
		PX4_ERR("unsupport fdcan index:%lu", index);
		return -1;
	}

	if (fdcan_bittiming(&_priv->arbi_timing) != OK) {
		printf("ERROR: Invalid CAN timings\n");
		return -1;
	}

	/* Configure the pins we're using to interface to the controller */

	stm32_configgpio(_priv->config->tx_pin);
	stm32_configgpio(_priv->config->rx_pin);

	/* Attach the fdcan interrupt handlers */

	if (irq_attach(_priv->config->mb_irq[0], fdcan_interrupt, NULL)) {
		/* We could not attach the ISR to the interrupt */
		PX4_ERR("ERROR: Failed to attach CAN RX IRQ\n");
		return -1;
	}

	if (irq_attach(_priv->config->mb_irq[1], fdcan_interrupt, NULL)) {
		/* We could not attach the ISR to the interrupt */
		PX4_ERR("ERROR: Failed to attach CAN TX IRQ\n");
		return -1;
	}

	/* Put the interface in the down state (disable interrupts, power off) */
	fdcan_reset(_priv);
	return 0;
}

int CanDriver::start()
{
	struct fdcan_driver_s *const priv = _priv;

	if (priv == nullptr) {
		PX4_ERR("can start priv == NULL\n");
		return -1;
	}

	uint32_t regval;

	irqstate_t flags = enter_critical_section();

	/* Reset the peripheral clock bus (only do this once) */
	if (!g_apb1h_init) {
		fdcan_apb1hreset();
		g_apb1h_init = true;
	}

	/* Exit Power-down / Sleep mode */

	fdcan_setenable(priv->base, 1);

	/* Enter Initialization mode */

	fdcan_setinit(priv->base, 1);

	/* Enter Configuration Changes Enabled mode */

	fdcan_setconfig(priv->base, 1);

	/* Disable interrupts while we configure the hardware */

	putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

	/* Compute CAN bit timings for this bitrate */

	/* Nominal / arbitration phase bitrate */

	if (fdcan_bittiming(&priv->arbi_timing) != OK) {
		fdcan_setinit(priv->base, 0);
		PX4_ERR("can bittiming fail\n");
		leave_critical_section(flags);
		return -EIO;
	}

	/* Set bit timings and prescalers (Nominal bitrate) */

	regval = ((priv->arbi_timing.sjw << FDCAN_NBTP_NSJW_SHIFT) | (priv->arbi_timing.bs1 << FDCAN_NBTP_NTSEG1_SHIFT) |
		  (priv->arbi_timing.bs2 << FDCAN_NBTP_TSEG2_SHIFT) | (priv->arbi_timing.prescaler << FDCAN_NBTP_NBRP_SHIFT));
	putreg32(regval, priv->base + STM32_FDCAN_NBTP_OFFSET);

	/* Be sure to fill data-phase register even if we're not using CAN FD */

	putreg32(regval, priv->base + STM32_FDCAN_DBTP_OFFSET);

	/* Operation Configuration */

	modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_FDOE, 0);

#if 0
	/* Disable Automatic Retransmission of frames upon error
	 * NOTE: This will even disable automatic retry due to lost arbitration!!
	 */

	modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_DAR);
#endif

	/* Configure Interrupts */

	/* Clear all interrupt flags
	 * Note: A flag is cleared by writing a 1 to the corresponding bit position
	 */

	putreg32(FDCAN_IR_MASK, priv->base + STM32_FDCAN_IR_OFFSET);

	/* Enable relevant interrupts */

	regval = FDCAN_IE_TCE	     /* Transmit Complete */
		 | FDCAN_IE_RF0NE  /* Rx FIFO 0 new message */
		 | FDCAN_IE_RF0FE  /* Rx FIFO 0 FIFO full */
		 | FDCAN_IE_RF1NE  /* Rx FIFO 1 new message */
		 | FDCAN_IE_RF1FE; /* Rx FIFO 1 FIFO full */
	putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

	/* Keep Rx interrupts on Line 0; move Tx to Line 1
	 * TC (Tx Complete) interrupt on line 1
	 */

	regval = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
	regval |= FDCAN_ILS_TCL;
	putreg32(FDCAN_ILS_TCL, priv->base + STM32_FDCAN_ILS_OFFSET);

	/* Enable Tx buffer transmission interrupts
	 * Note: Still need fdcan_enable_interrupts() to set ILE (IR line enable)
	 */

	putreg32(FDCAN_TXBTIE_TIE, priv->base + STM32_FDCAN_TXBTIE_OFFSET);

	/* Configure Message RAM
	 *
	 * The available 2560 words (10 kiB) of RAM are shared between both FDCAN
	 * interfaces. It is up to us to ensure each interface has its own non-
	 * overlapping region of RAM assigned to it by properly assigning the start
	 * and end addresses for all regions of RAM.
	 *
	 * We will give each interface half of the available RAM.
	 *
	 * Rx buffers are only used in conjunction with acceptance filters; we
	 * don't have any specific need for this, so we will only use Rx FIFOs.
	 *
	 * Each FIFO can hold up to 64 elements, where each element (for a classic
	 * CAN 2.0B frame) is up to 4 words long (8 bytes data + header bits)
	 *
	 * Let's make use of the full 64 FIFO elements for FIFO0.  We have no need
	 * to separate messages between FIFO0 and FIFO1, so ignore FIFO1 for
	 * simplicity.
	 *
	 * Note that the start addresses given to FDCAN are in terms of _words_,
	 * not  bytes, so when we go to read/write to/from the message RAM, there
	 * will be a factor of 4 necessary in the address relative to the SA
	 * register values.
	 */

	/* Location of this interface's message RAM - address in CPU memory address
	 * and relative address (in words) used for configuration
	 */

	const uint32_t iface_ram_base = (2560 / 2) * priv->iface_idx;
	const uint32_t gl_ram_base = STM32_CANRAM_BASE;
	uint32_t ram_offset = iface_ram_base;

	/* Standard ID Filters: Allow space for 128 filters (128 words) */

	const uint8_t n_stdid = 0;
	priv->message_ram.filt_stdid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

	regval = (n_stdid << FDCAN_SIDFC_LSS_SHIFT) & FDCAN_SIDFC_LSS_MASK;
	regval |= ram_offset << FDCAN_SIDFC_FLSSA_SHIFT;
	putreg32(regval, priv->base + STM32_FDCAN_SIDFC_OFFSET);
	ram_offset += n_stdid;

	/* Extended ID Filters: Allow space for 128 filters (128 words) */

	const uint8_t n_extid = 0;
	priv->message_ram.filt_extid_addr = gl_ram_base + ram_offset * WORD_LENGTH;

	regval = (n_extid << FDCAN_XIDFC_LSE_SHIFT) & FDCAN_XIDFC_LSE_MASK;
	regval |= ram_offset << FDCAN_XIDFC_FLESA_SHIFT;
	putreg32(regval, priv->base + STM32_FDCAN_XIDFC_OFFSET);
	ram_offset += n_extid;

	/* Set size of each element in the Rx/Tx buffers and FIFOs */
	putreg32(0, priv->base + STM32_FDCAN_RXESC_OFFSET); /* 8 byte space for every element (Rx buffer, FIFO1, FIFO0) */
	putreg32(0, priv->base + STM32_FDCAN_TXESC_OFFSET); /* 8 byte space for every element (Tx buffer) */

	priv->message_ram.n_rxfifo0 = NUM_RX_FIFO0;
	priv->message_ram.n_rxfifo1 = NUM_RX_FIFO1;
	priv->message_ram.n_txfifo = NUM_TX_FIFO;

	/* Assign Rx Mailbox pointer in the driver structure */

	priv->message_ram.rxfifo0_addr = gl_ram_base + ram_offset * WORD_LENGTH;
	priv->rx = (struct rx_fifo_s *)(priv->message_ram.rxfifo0_addr);

	/* Set Rx FIFO0 size (64 elements max) */

	regval = (ram_offset << FDCAN_RXF0C_F0SA_SHIFT) & FDCAN_RXF0C_F0SA_MASK;
	regval |= (NUM_RX_FIFO0 << FDCAN_RXF0C_F0S_SHIFT) & FDCAN_RXF0C_F0S_MASK;
	putreg32(regval, priv->base + STM32_FDCAN_RXF0C_OFFSET);
	ram_offset += NUM_RX_FIFO0 * FIFO_ELEMENT_SIZE;

	/* Not using Rx FIFO1 */

	/* Assign Tx Mailbox pointer in the driver structure */

	priv->message_ram.txfifo_addr = gl_ram_base + ram_offset * WORD_LENGTH;
	priv->tx = (struct tx_fifo_s *)(priv->message_ram.txfifo_addr);

	/* Set Tx FIFO size (32 elements max) */

	regval = (NUM_TX_FIFO << FDCAN_TXBC_TFQS_SHIFT) & FDCAN_TXBC_TFQS_MASK;
	regval &= ~FDCAN_TXBC_TFQM; /* Use FIFO */
	regval |= (ram_offset << FDCAN_TXBC_TBSA_SHIFT) & FDCAN_TXBC_TBSA_MASK;
	putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);

	/* Default filter configuration - Accept all messages into Rx FIFO0 */

	regval = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
	regval &= ~FDCAN_GFC_ANFS; /* Accept non-matching stdid frames into FIFO0 */
	regval &= ~FDCAN_GFC_ANFE; /* Accept non-matching extid frames into FIFO0 */
	putreg32(regval, priv->base + STM32_FDCAN_GFC_OFFSET);

	/* Enable interrupts (at both device and NVIC level) */

	fdcan_enable_interrupts(priv);

	/* Leave init mode */

	fdcan_setinit(priv->base, 0);

	leave_critical_section(flags);
	return 0;
}
/****************************************************************************
 * Function: stop
 *
 * Description:
 *   Put the device in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The device has previously been initialized, including message RAM
 ****************************************************************************/

int CanDriver::stop()
{
	fdcan_reset(_priv);
	return 0;
}

int16_t CanDriver::send(const can_frame &frame)
{
	struct fdcan_driver_s *const priv = _priv;

	if (priv == nullptr) {
		return -ENODEV;
	}

	irqstate_t flags = enter_critical_section();

	/* First, check if there are any slots available in the queue */

	uint32_t regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

	if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		/* Tx FIFO / Queue is full */
		leave_critical_section(flags);
		return -EBUSY;
	}

	/* Next, get the next available FIFO index from the controller */

	regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
	const uint8_t mbi = (regval & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_SHIFT;

	/* Now, we can copy the CAN frame to the FIFO (in message RAM) */

	if (mbi >= NUM_TX_FIFO) {
		PX4_ERR("Invalid Tx mailbox index encountered in transmit\n");
		leave_critical_section(flags);
		return -EIO;
	}

	struct tx_fifo_s *mb = &priv->tx[mbi];

	/* Setup timeout deadline if enabled */
	/* Attempt to write frame */

	union tx_fifo_header_u header;

	if (frame.can_id & CAN_EFF_FLAG) {
		header.id.xtd = 1;
		header.id.extid = frame.can_id & CAN_EFF_MASK;

	} else {
		header.id.xtd = 0;
		header.id.stdid = frame.can_id & CAN_SFF_MASK;
	}

	header.id.esi = frame.can_id & CAN_ERR_FLAG ? 1 : 0;
	header.id.rtr = frame.can_id & CAN_RTR_FLAG ? 1 : 0;
	header.dlc = frame.can_dlc;
	header.brs = 0;  /* No bitrate switching */
	header.fdf = 0;  /* Classic CAN frame, not CAN-FD */
	header.efc = 0;  /* Don't store Tx events */
	header.mm = mbi; /* Mailbox Marker for our own use; just store FIFO index */

	/* Store into message RAM */

	mb->header.w0 = header.w0;
	mb->header.w1 = header.w1;
	memcpy(mb->data, frame.data, frame.can_dlc);

	/* GO - Submit the transmission request for this element */

	putreg32(1 << mbi, priv->base + STM32_FDCAN_TXBAR_OFFSET);
	leave_critical_section(flags);

	/* Increment statistics */
	priv->txmb[mbi].pending = TX_BUSY;
#if JOICAN_DEBUG==1
	priv->_send_count++;
#endif
	return OK;
}
int16_t CanDriver::send(const can_frame *frame, int num)
{
	struct fdcan_driver_s *const priv = _priv;

	if (priv == nullptr) {
		return -ENODEV;
	}

	if (num < 1) {
		return -EIO;
	}

	irqstate_t flags = enter_critical_section();

	/* First, check if there are any slots available in the queue */

	uint32_t regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

	if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		/* Tx FIFO / Queue is full */
		leave_critical_section(flags);
		return -EBUSY;
	}

	/* Next, get the next available FIFO index from the controller */

	regval = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
	const uint8_t mbi = (regval & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_SHIFT;
	const uint8_t tffl = (regval & FDCAN_TXFQS_TFFL) >> FDCAN_TXFQS_TFFL_SHIFT;

	if (tffl < num) {
		//no enough space
		leave_critical_section(flags);
		return -EIO;
	}

	uint32_t txbar_put = 0;

	for (int i = 0; i < num; ++i) {
		/* Now, we can copy the CAN frame to the FIFO (in message RAM) */
		const uint8_t mbinow = mbi + i;

		if (mbinow >= NUM_TX_FIFO) {
			PX4_ERR("Invalid Tx mailbox index encountered in transmit\n");
			leave_critical_section(flags);
			return -EIO;
		}

		struct tx_fifo_s *mb = &priv->tx[mbinow];

		/* Attempt to write frame */

		union tx_fifo_header_u header;

		if (frame[i].can_id & CAN_EFF_FLAG) {
			header.id.xtd = 1;
			header.id.extid = frame[i].can_id & CAN_EFF_MASK;

		} else {
			header.id.xtd = 0;
			header.id.stdid = frame[i].can_id & CAN_SFF_MASK;
		}

		header.id.esi = frame[i].can_id & CAN_ERR_FLAG ? 1 : 0;
		header.id.rtr = frame[i].can_id & CAN_RTR_FLAG ? 1 : 0;
		header.dlc = frame[i].can_dlc;
		header.brs = 0;  /* No bitrate switching */
		header.fdf = 0;  /* Classic CAN frame, not CAN-FD */
		header.efc = 0;  /* Don't store Tx events */
		header.mm = mbinow; /* Mailbox Marker for our own use; just store FIFO index */

		/* Store into message RAM */

		mb->header.w0 = header.w0;
		mb->header.w1 = header.w1;
		memcpy(mb->data, frame[i].data, frame[i].can_dlc);

		txbar_put |= 1 << mbinow;

		/* Increment statistics */
		priv->txmb[mbinow].pending = TX_BUSY;
	}

	putreg32(txbar_put, priv->base + STM32_FDCAN_TXBAR_OFFSET);
	leave_critical_section(flags);
#if JOICAN_DEBUG==1
	priv->_send_count += num;
#endif
	return OK;
}
int16_t CanDriver::receive(can_frame &out_frame)
{
	return _priv->rx_buf.pop(out_frame);
}
int CanDriver::waitReceive()
{
	return sem_wait(&_priv->receive_sem);
}
bool CanDriver::isFifoFull()
{
	/* Check if the Tx queue is full */
	uint32_t regval = getreg32(_priv->base + STM32_FDCAN_TXFQS_OFFSET);

	if ((regval & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		return true; /* Sorry, out of room, try back later */
	}

	return false;
}
}  // namespace joican
