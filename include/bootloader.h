/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

This file is part of vectorcontrol.

vectorcontrol is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
vectorcontrol. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

enum can_bootloader_message_id_t {
    CAN_BOOTLOADER_REQUEST = 0x720,
    CAN_BOOTLOADER_ANNOUNCE = 0x721,
    CAN_BOOTLOADER_WRITE_PAGE_START = 0x722,
    CAN_BOOTLOADER_WRITE_PAGE_DATA = 0x723,
    CAN_BOOTLOADER_WRITE_PAGE_RESPONSE = 0x724,
    CAN_BOOTLOADER_RESTART = 0x725
};

enum can_bootloader_status_t {
    BOOTLOADER_STATUS_OK = 0,
    BOOTLOADER_STATUS_WRONG_STATE = 1,
    BOOTLOADER_STATUS_WRONG_ADDRESS = 2,
    BOOTLOADER_STATUS_MISSING_DATA = 3,
    BOOTLOADER_STATUS_INVALID_CRC = 4,
    BOOTLOADER_STATUS_UNMODIFIED = 5
};

struct can_bootloader_write_page_start_t {
    uint16_t page_index;
    uint16_t transfer_words; /* 0-512 */
    uint32_t transfer_crc;
} __attribute__ ((packed));

struct can_bootloader_write_page_data_t {
    uint16_t word_index; /* 0-512 */
    uint32_t word;
} __attribute__ ((packed));

struct can_bootloader_write_page_response_t {
    uint32_t page_crc;
    uint8_t status;
} __attribute__ ((packed));

extern "C" void bootloader_start(void);
