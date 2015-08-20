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
#include <uavcan/data_type.hpp>
#include "shared.h"


static uint64_t bootloader_calculate_signature(
    const bootloader_app_shared_t *pshared
);

static uint64_t bootloader_calculate_signature(
    const bootloader_app_shared_t *pshared
) {
    uavcan::DataTypeSignatureCRC crc;
    crc.add((uint8_t*)(&pshared->signature), sizeof(uint32_t) * 3u);
    return crc.get();
}


/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

#define crc_HiLOC       STM32_CAN1_FIR(2,1)
#define crc_LoLOC       STM32_CAN1_FIR(2,2)
#define signature_LOC   STM32_CAN1_FIR(3,1)
#define bus_speed_LOC   STM32_CAN1_FIR(3,2)
#define node_id_LOC     STM32_CAN1_FIR(4,1)
#define CRC_H 1
#define CRC_L 0


bool bootloader_read(bootloader_app_shared_t *shared) {
    shared->signature = getreg32(signature_LOC);
    shared->bus_speed = getreg32(bus_speed_LOC);
    shared->node_id = getreg32(node_id_LOC);
    shared->crc.ul[CRC_L] = getreg32(crc_LoLOC);
    shared->crc.ul[CRC_H] = getreg32(crc_HiLOC);

    if (shared->crc.ull == bootloader_calculate_signature(shared)) {
        return true;
    } else {
        return false;
    }
}


void bootloader_write(const bootloader_app_shared_t *shared) {
    bootloader_app_shared_t working = *shared;

    working.signature = BOOTLOADER_COMMON_APP_SIGNATURE;
    working.crc.ull = bootloader_calculate_signature(&working);

    putreg32(working.signature, signature_LOC);
    putreg32(working.bus_speed, bus_speed_LOC);
    putreg32(working.node_id, node_id_LOC);
    putreg32(working.crc.ul[CRC_L], crc_LoLOC);
    putreg32(working.crc.ul[CRC_H], crc_HiLOC);
}
