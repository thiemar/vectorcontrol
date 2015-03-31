/*
Copyright (c) 2015 Ben Dyer <ben_dyer@mac.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
