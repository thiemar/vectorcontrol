/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include <stm32.h>
#include "can.h"


#define INAK_TIMEOUT          65535
#define SJW_POS               24
#define BS1_POS               16
#define BS2_POS               20

#define QUANTA 9

#define CAN_1MBAUD_SJW 0
#define CAN_1MBAUD_BS1 6u
#define CAN_1MBAUD_BS2 0
#define CAN_1MBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/1000000/QUANTA)

#define CAN_500KBAUD_SJW 0
#define CAN_500KBAUD_BS1 6u
#define CAN_500KBAUD_BS2 0
#define CAN_500KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/500000/QUANTA)

#define CAN_250KBAUD_SJW 0
#define CAN_250KBAUD_BS1 6u
#define CAN_250KBAUD_BS2 0
#define CAN_250KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/250000/QUANTA)

#define CAN_125KBAUD_SJW 0
#define CAN_125KBAUD_BS1 6u
#define CAN_125KBAUD_BS2 0
#define CAN_125KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/125000/QUANTA)

#define CAN_BTR_LBK_SHIFT 30
#define CAN_TSR_RQCP_SHFTS    8



static can_speed_t can_freq2speed_(int freq)
{
    return freq == 1000000u ? CAN_1MBAUD :
           freq ==  500000u ? CAN_500KBAUD :
           freq ==  250000u ? CAN_250KBAUD :
                              CAN_125KBAUD;
}


int can_init(int freq)
{
    int speedndx = can_freq2speed_(freq) - 1u;

    const uint32_t bitrates[] = {
        (CAN_125KBAUD_SJW << SJW_POS) |
        (CAN_125KBAUD_BS1 << BS1_POS) |
        (CAN_125KBAUD_BS2 << BS2_POS) | (CAN_125KBAUD_PRESCALER - 1),

        (CAN_250KBAUD_SJW << SJW_POS) |
        (CAN_250KBAUD_BS1 << BS1_POS) |
        (CAN_250KBAUD_BS2 << BS2_POS) | (CAN_250KBAUD_PRESCALER - 1),

        (CAN_500KBAUD_SJW << SJW_POS) |
        (CAN_500KBAUD_BS1 << BS1_POS) |
        (CAN_500KBAUD_BS2 << BS2_POS) | (CAN_500KBAUD_PRESCALER - 1),

        (CAN_1MBAUD_SJW   << SJW_POS) |
        (CAN_1MBAUD_BS1   << BS1_POS) |
        (CAN_1MBAUD_BS2   << BS2_POS) | (CAN_1MBAUD_PRESCALER - 1)
    };

    uint32_t timeout;
    /*
     *  Reset state is 0x0001 0002 CAN_MCR_DBF|CAN_MCR_SLEEP
     *  knock down Sleep and raise CAN_MCR_INRQ
     */

    putreg32(CAN_MCR_DBF | CAN_MCR_INRQ, STM32_CAN1_MCR);

    /* Wait until initialization mode is acknowledged */

    for (timeout = INAK_TIMEOUT; timeout > 0; timeout--) {
        if ((getreg32(STM32_CAN1_MSR) & CAN_MSR_INAK) != 0) {
            /* We are in initialization mode */
            break;
        }
    }

    if (timeout < 1) {
        /*
         * Initialization failed, not much we can do now other than try a normal
         * startup. */
        return -1;
    }


    putreg32(bitrates[speedndx] | CAN_Mode_Normal << CAN_BTR_LBK_SHIFT, STM32_CAN1_BTR);
    putreg32(CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_DBF | CAN_MCR_TXFP, STM32_CAN1_MCR);

    for (timeout = INAK_TIMEOUT; timeout > 0; timeout--) {
        if ((getreg32(STM32_CAN1_MSR) & CAN_MSR_INAK) == 0) {
            /* We are in initialization mode */
            break;
        }
    }

    if (timeout < 1) {
        return -1;
    } else {
        return 0;
    }
}


bool can_is_ready(uint8_t mailbox) {
    return getreg32(STM32_CAN1_TSR) & (CAN_TSR_TME0 << mailbox);
}


void can_tx(
    uint8_t mailbox,
    uint32_t message_id,
    size_t length,
    const uint8_t *message
) {
    uint32_t data[2];

    memcpy(data, message, sizeof(data));

    uint32_t mask = CAN_TSR_TME0 << mailbox;
    while ((getreg32(STM32_CAN1_TSR) & mask) == 0);

    /*
     * To allow detection of completion  - Set the LEC to
     * 'No error' state off all 1s
     */

    putreg32(CAN_ESR_LEC_MASK, STM32_CAN1_ESR);

    putreg32(length & CAN_TDTR_DLC_MASK, STM32_CAN1_TDTR(mailbox));
    putreg32(data[0], STM32_CAN1_TDLR(mailbox));
    putreg32(data[1], STM32_CAN1_TDHR(mailbox));
    putreg32((message_id << CAN_TIR_EXID_SHIFT) | CAN_TIR_IDE | CAN_TIR_TXRQ,
         STM32_CAN1_TIR(mailbox));
}


bool can_rx(
    uint8_t fifo,
    uint8_t *filter_id,
    uint32_t *message_id,
    size_t *length,
    uint8_t *message
) {
    uint32_t data[2];
    const uint32_t fifos[] = { STM32_CAN1_RF0R, STM32_CAN1_RF1R };

    if (getreg32(fifos[fifo & 1]) & CAN_RFR_FMP_MASK) {
        *message_id = (getreg32(STM32_CAN1_RIR(fifo)) & CAN_RIR_EXID_MASK) >>
                      CAN_RIR_EXID_SHIFT;
        *length = (getreg32(STM32_CAN1_RDTR(fifo)) & CAN_RDTR_DLC_MASK) >>
                  CAN_RDTR_DLC_SHIFT;
        *filter_id = (getreg32(STM32_CAN1_RDTR(fifo)) & CAN_RDTR_FM_MASK) >>
                     CAN_RDTR_FM_SHIFT;
        data[0] = getreg32(STM32_CAN1_RDLR(fifo));
        data[1] = getreg32(STM32_CAN1_RDHR(fifo));

        putreg32(CAN_RFR_RFOM, fifos[fifo & 1]);
        memcpy(message, data, sizeof(data));

        return true;
    } else {
        return false;
    }
}


void can_set_dtid_filter(
    uint8_t fifo,
    uint8_t filter_id,
    bool is_service,
    uint16_t dtid,
    uint8_t node_id
) {
    uint32_t mask;
    mask = (1u << filter_id);

    /* Enable filter init mode */
    putreg32(CAN_FMR_FINIT, STM32_CAN1_FMR);
    /* Disable this filter */
    putreg32(getreg32(STM32_CAN1_FA1R) & ~mask, STM32_CAN1_FA1R);
    /* Enable 32-bit filter mode for this filter */
    putreg32(getreg32(STM32_CAN1_FS1R) | mask, STM32_CAN1_FS1R);

    /* Set the filter ID based on frame type */
    if (is_service) { /* It's a service request */
        putreg32(((dtid << 16u) | 0x8000u | (node_id << 0x8u) | 0x80u) <<
                    CAN_RIR_EXID_SHIFT,
                 STM32_CAN1_FIR(filter_id, 1));
    } else { /* It's a message */
        putreg32((dtid << 8u) << CAN_RIR_EXID_SHIFT,
                 STM32_CAN1_FIR(filter_id, 1));
    }

    /*
    For messages, this mask matches DTID and "service not message" flag.
    For service requests, it matches DTID, "request not response" flag,
    destination node ID, and "service not message" flags.
    */
    putreg32(0x00FFFF80u << CAN_RIR_EXID_SHIFT, STM32_CAN1_FIR(filter_id, 2));


    /* Set this filter to mask mode */
    putreg32(getreg32(STM32_CAN1_FM1R) & ~mask, STM32_CAN1_FM1R);

    if (fifo) {
        /* Set this filter to FIFO 1 */
        putreg32(getreg32(STM32_CAN1_FFA1R) | mask, STM32_CAN1_FFA1R);
    } else {
        /* Set this filter to FIFO 0 */
        putreg32(getreg32(STM32_CAN1_FFA1R) & ~mask, STM32_CAN1_FFA1R);
    }

    /* Enable this filter */
    putreg32(getreg32(STM32_CAN1_FA1R) | mask, STM32_CAN1_FA1R);

    /* Clear FINIT to leave initialization mode */
    putreg32(0, STM32_CAN1_FMR);
}
