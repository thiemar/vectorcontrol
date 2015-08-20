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

#pragma once

typedef enum {
    CAN_UNKNOWN   = 0,
    CAN_125KBAUD  = 1,
    CAN_250KBAUD  = 2,
    CAN_500KBAUD  = 3,
    CAN_1MBAUD    = 4
} can_speed_t;


typedef enum {
    CAN_Mode_Normal = 0,         // Bits 30 and 31 00
    CAN_Mode_LoopBack = 1,       // Bit 30: Loop Back Mode (Debug)
    CAN_Mode_Silent = 2,         // Bit 31: Silent Mode (Debug)
    CAN_Mode_Silent_LoopBack = 3 // Bits 30 and 31 11
} can_mode_t;


int can_init(int freq);
bool can_is_ready(uint8_t mailbox);
void can_tx(
    uint8_t mailbox,
    uint32_t message_id,
    size_t length,
    const uint8_t *message
);
bool can_rx(
    uint8_t fifo,
    uint8_t *filter_id,
    uint32_t *message_id,
    size_t *length,
    uint8_t *message
);
void can_set_dtid_filter(
    uint8_t fifo,
    uint8_t filter_id,
    bool is_service,
    uint16_t dtid,
    uint8_t node_id
);
