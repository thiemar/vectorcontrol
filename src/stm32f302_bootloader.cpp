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

#include <stdint.h>
#include <stdlib.h>
#include "stm32f30x.h"
#include "cmsis_device.h"
#include "bootloader.h"


extern "C" void _start(void);


#define PORT_CAN_SILENT GPIOB
#define PIN_CAN_SILENT GPIO_Pin_6
#define PIN_CAN_SILENT_INDEX 6

#define PORT_CAN_RX GPIOA
#define PIN_CAN_RX GPIO_Pin_11
#define PIN_CAN_RX_INDEX 11

#define PORT_CAN_TX GPIOA
#define PIN_CAN_TX GPIO_Pin_12
#define PIN_CAN_TX_INDEX 12


/* 1 Mbaud from 72 MHz system clock */
#define CAN_PRESCALER 4u
#define CAN_SJW 0
#define CAN_BS1 6u
#define CAN_BS2 0
#define CAN_INAK_TIMEOUT 100000
#define CAN_REQUEST_TIMEOUT 1000000


#define FLASH_PAGE_BYTES 2048u
#define FLASH_PAGE_WORDS (FLASH_PAGE_BYTES / sizeof(uint32_t))
#define FLASH_NUM_PAGES 26


static uint32_t page_buf[FLASH_PAGE_WORDS];


static void
__attribute__ ((section(".bootloader")))
wait_(void) {
    while ((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);

    if ((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR) != (uint32_t)0x00) {
        abort();
    }
    if ((FLASH->SR & (uint32_t)FLASH_SR_PGERR) != (uint32_t)0x00) {
        abort();
    }
}


static bool
__attribute__ ((section(".bootloader")))
has_flash_page_changed_(uint8_t page, uint32_t* buf) {
    size_t i;
    uint32_t* flash;

    flash = (uint32_t*)(FLASH_BASE + FLASH_PAGE_BYTES * (uint32_t)page);
    for (i = 0; i < FLASH_PAGE_WORDS; i++) {
        if (flash[i] != buf[i]) {
            return true;
        }
    }

    return false;
}


static bool
__attribute__ ((section(".bootloader")))
write_flash_page_(uint8_t page, uint32_t* buf) {
    uint32_t addr;
    size_t i;

    addr = FLASH_BASE + FLASH_PAGE_BYTES * (uint32_t)page;

    /* FLASH_Unlock(); */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* FLASH_ErasePage(addr); */
    wait_();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = addr;
    FLASH->CR |= FLASH_CR_STRT;

    wait_();

    FLASH->CR &= ~FLASH_CR_PER;

    /* FLASH_ProgramWord(addr, word); */
    FLASH->CR |= FLASH_CR_PG;
    for (i = 0; i < FLASH_PAGE_WORDS; i++) {
        *(__IO uint16_t*)addr = (uint16_t)buf[i];
        addr += 2u;

        wait_();

        *(__IO uint16_t*)addr = (uint16_t)(buf[i] >> 16);
        addr += 2u;

        wait_();

        /* Make sure the word was written correctly */
        if (*(volatile uint32_t*)(addr - 4u) != buf[i]) {
            break;
        }
    }
    FLASH->CR &= ~FLASH_CR_PG;

    /* FLASH_Lock(); */
    FLASH->CR |= FLASH_CR_LOCK;

    /* Loop ended early -- a word must have been written incorrectly */
    if (i != FLASH_PAGE_WORDS) {
        abort();
    }

    return true;
}

static uint32_t
__attribute__ ((section(".bootloader")))
adler32_(size_t length, const void *buf) {
    const uint32_t MOD_ADLER = 65521u;
    uint8_t *data = (uint8_t*)buf;
    uint32_t a = 1u, b = 0;
    size_t index;

    /* Process each byte of the data in order */
    for (index = 0; index < length; index++)
    {
        a = (a + data[index]) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }

    return (b << 16u) | a;
}


static void
__attribute__ ((section(".bootloader")))
can_tx_(
    uint32_t message_id,
    size_t length,
    const uint8_t *message
) {
    CAN1->sTxMailBox[0].TIR = 0;
    CAN1->sTxMailBox[0].TIR |= (message_id & 0x7FFu) << 21u;
    CAN1->sTxMailBox[0].TDTR = length & 0xFu;
    CAN1->sTxMailBox[0].TDLR = (message[3] << 24) | (message[2] << 16) |
                               (message[1] << 8) | (message[0]);
    CAN1->sTxMailBox[0].TDHR = (message[7] << 24) | (message[6] << 16) |
                               (message[5] << 8) | (message[4]);
    CAN1->sTxMailBox[0].TIR |= 1u;
}


static bool
__attribute__ ((section(".bootloader")))
can_rx_(
    uint32_t& out_message_id,
    size_t& out_length,
    uint8_t *out_message
) {
    uint8_t fifo;

    /* Check if a message is pending */
    if (CAN1->RF0R & 3u) {
        fifo = 0;
    } else if (CAN1->RF1R & 3u) {
        fifo = 1;
    } else {
        return false;
    }

    /* If so, process it */
    out_message_id = (CAN1->sFIFOMailBox[fifo].RIR >> 21u) & 0x7FFu;
    out_length = CAN1->sFIFOMailBox[fifo].RDTR & 0xFu;
    out_message[0] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDLR >> 0) & 0xFFu);
    out_message[1] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDLR >> 8) & 0xFFu);
    out_message[2] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDLR >> 16) & 0xFFu);
    out_message[3] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDLR >> 24) & 0xFFu);
    out_message[4] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDHR >> 0) & 0xFFu);
    out_message[5] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDHR >> 8) & 0xFFu);
    out_message[6] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDHR >> 16) & 0xFFu);
    out_message[7] = (uint8_t)((CAN1->sFIFOMailBox[fifo].RDHR >> 24) & 0xFFu);

    /* Release the message from the receive FIFO */
    if (fifo == 0) {
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    } else {
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }

    return true;
}


extern "C" void
__attribute__ ((section(".bootloader_start"),externally_visible))
bootloader_start(void) {
    /*
    See implementation in stm32f302_hal.cpp -- this writes registers directly
    to save code space.

    We enable the minimum necessary to get CAN set up:
    * GPIOA
    * GPIOB
    * CAN1
    * SYSCFG
    */
    size_t write_page_index, write_word_index, write_page_words;
    uint32_t write_page_crc, crc;

    uint32_t offset, timer;
    enum {
        AWAITING_REQUEST,
        AWAITING_PAGE_START,
        AWAITING_PAGE_DATA
    } state;

    union {
        uint8_t bytes[8];
        struct can_bootloader_write_page_start_t start;
        struct can_bootloader_write_page_data_t data;
        struct can_bootloader_write_page_response_t response;
    } message;
    size_t message_length;
    uint32_t message_id;
    bool got_message;

    /* Low-level hardware initialization (including clock setup) */
    SystemInit();

    /* Enable FPU */
    SCB->CPACR |= (0xF << 20);

    /* RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); */
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA;

    /* RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); */
    RCC->AHBENR |= RCC_AHBPeriph_GPIOB;

    /* RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); */
    RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;

    /* RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); */
    RCC->APB1ENR |= RCC_APB1Periph_CAN1;

    /* GPIO_SetBits(PORT_CAN_SILENT, PIN_CAN_SILENT.GPIO_Pin); */
    PORT_CAN_SILENT->BSRR = PIN_CAN_SILENT;

    /*
    Configure RX and TX pins with:
    GPIO_Mode_AF
    GPIO_Speed_Level_2
    GPIO_OType_PP
    GPIO_PuPd_NOPULL

    Configure PIN_CAN_SILENT as above but GPIO_Mode_OUT.
    */

    /* GPIO_Init(PORT_CAN_TX, PIN_CAN_TX); */
    offset = PIN_CAN_TX_INDEX * 2u;
    PORT_CAN_TX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_TX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_TX->OTYPER = (uint16_t)
        (PORT_CAN_TX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_TX_INDEX));
    PORT_CAN_TX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_TX_INDEX);
    PORT_CAN_TX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_TX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_TX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_TX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_RX, PIN_CAN_RX); */
    offset = PIN_CAN_RX_INDEX * 2u;
    PORT_CAN_RX->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_RX->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_RX->OTYPER = (uint16_t)
        (PORT_CAN_RX->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_RX_INDEX));
    PORT_CAN_RX->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_RX_INDEX);
    PORT_CAN_RX->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_RX->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_RX->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_RX->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_Init(PORT_CAN_SILENT, PIN_CAN_SILENT); */
    offset = PIN_CAN_SILENT_INDEX * 2u;
    PORT_CAN_SILENT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << offset);
    PORT_CAN_SILENT->OSPEEDR |= GPIO_Speed_Level_2 << offset;
    PORT_CAN_SILENT->OTYPER = (uint16_t)
        (PORT_CAN_SILENT->OTYPER & ~(GPIO_OTYPER_OT_0 << PIN_CAN_SILENT_INDEX));
    PORT_CAN_SILENT->OTYPER |= (uint16_t)(GPIO_OType_PP << PIN_CAN_SILENT_INDEX);
    PORT_CAN_SILENT->MODER &= ~(GPIO_MODER_MODER0 << offset);
    PORT_CAN_SILENT->MODER |= GPIO_Mode_AF << offset;
    PORT_CAN_SILENT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << offset);
    PORT_CAN_SILENT->PUPDR |= GPIO_PuPd_NOPULL << offset;

    /* GPIO_PinAFConfig(PORT_CAN_RX, GPIO_PinSource11, GPIO_AF_9); */
    offset = (GPIO_PinSource11 & 7u) * 4u;
    PORT_CAN_RX->AFR[GPIO_PinSource11 >> 3u] &= ~(0xFu << offset);
    PORT_CAN_RX->AFR[GPIO_PinSource11 >> 3u] |= GPIO_AF_9 << offset;

    /* GPIO_PinAFConfig(PORT_CAN_TX, GPIO_PinSource12, GPIO_AF_9); */
    offset = (GPIO_PinSource12 & 7u) * 4u;
    PORT_CAN_TX->AFR[GPIO_PinSource12 >> 3u] &= ~(0xFu << offset);
    PORT_CAN_TX->AFR[GPIO_PinSource12 >> 3u] |= GPIO_AF_9 << offset;


    /* CAN setup */

    /* CAN_DeInit(CAN1); */
    RCC->APB1RSTR |= RCC_APB1Periph_CAN1;
    RCC->APB1RSTR &= ~RCC_APB1Periph_CAN1;

    /* CAN_Init(CAN1, &config); */
    CAN1->MCR = (CAN1->MCR & (~CAN_MCR_SLEEP)) | CAN_MCR_INRQ;

    timer = 0;
    while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK &&
            timer != CAN_INAK_TIMEOUT) {
        timer++;
    }

    if ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {
        /*
        Initialization failed, not much we can do now other than try a normal
        startup.
        */
        goto startup;
    }

    CAN1->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);
    CAN1->MCR |= (CAN_MCR_ABOM | CAN_MCR_AWUM);
    CAN1->BTR = (CAN_Mode_Normal << 30) | (CAN_SJW << 24) | (CAN_BS1 << 16) |
                (CAN_BS2 << 20) | (CAN_PRESCALER - 1);

    /* Leave initialization mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;

    timer = 0;
    while ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK &&
            timer != CAN_INAK_TIMEOUT) {
        timer++;
    }

    if ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {
        /* Initialization failed; try to start up normally. */
        goto startup;
    }

    /* CAN filter initialization -- accept everything */
    CAN1->FMR |= 1u;
    CAN1->FA1R &= ~1u;
    CAN1->FS1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FM1R &= ~1u;
    CAN1->FFA1R &= ~1u;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~1u;

    /*
    * Wait up to 0.5 (ish) s for CAN_BOOTLOADER_REQUEST; if not received in
      that time, start up in user mode.
    * Send CAN_BOOTLOADER_ANNOUNCE.
    * Wait for CAN_BOOTLOADER_WRITE_PAGE_START to be received.
    * Store the page number (2 bytes), transfer length (1 byte) and page
      checksum (CRC32, 4 bytes).
    * Enter a loop in which 4-byte firmware data packets are received and
      copied to a buffer in RAM. Once transfer length bytes have been received
      the data checksum is compared with the provided page checksum; if they
      match, a confirmation CAN message is transmitted.
    * The buffer data is compared with the flash data for that page. If there
      are differences, the page is erased, the new data is written, and
      another confirmation message is sent. Otherwise, a no-change
      confirmation message is sent.
    * After each page is written, the system awaits another firmware update
      address packet, or a restart command packet. If a firmware update
      address packet is received, the system continues as above. If a restart
      packet is received, the system restarts.
    */
    timer = 0;
    state = AWAITING_REQUEST;
    write_word_index = 0xFFFFFFFFu;
    write_page_index = 0xFFFFFFFFu;
    write_page_crc = 0xFFFFFFFFu;
    write_page_words = 0xFFFFFFFFu;
    while (true) {
        timer++;

        got_message = can_rx_(message_id, message_length, message.bytes);
        if (!got_message) {
            if (state == AWAITING_REQUEST && timer >= CAN_REQUEST_TIMEOUT) {
                /* Bootloader timed out; continue with startup */
                goto startup;
            } else {
                /* Keep waiting */
                continue;
            }
        }

        if (state == AWAITING_REQUEST && message_id == CAN_BOOTLOADER_REQUEST) {
            state = AWAITING_PAGE_START;

            /* Enable CAN transmit */
            /* GPIO_ResetBits(PORT_CAN_SILENT, PIN_CAN_SILENT.GPIO_Pin); */
            PORT_CAN_SILENT->BRR = PIN_CAN_SILENT;

            /* Transmit confirmation */
            can_tx_(CAN_BOOTLOADER_ANNOUNCE, 0, message.bytes);
        } else if ((state == AWAITING_PAGE_START || state == AWAITING_PAGE_DATA) &&
                   message_id == CAN_BOOTLOADER_WRITE_PAGE_START) {
            if (message.start.page_index >= FLASH_NUM_PAGES ||
                    message.start.transfer_words > FLASH_PAGE_WORDS ||
                    message.start.transfer_words == 0) {
                /* Error */
                message.response.page_crc = 0;
                message.response.status =
                    (uint8_t)BOOTLOADER_STATUS_WRONG_ADDRESS;
                can_tx_(CAN_BOOTLOADER_WRITE_PAGE_RESPONSE,
                        sizeof(struct can_bootloader_write_page_response_t),
                        message.bytes);
                state = AWAITING_PAGE_START;
            } else {
                state = AWAITING_PAGE_DATA;
                write_page_index = message.start.page_index;
                write_page_words = message.start.transfer_words;
                write_word_index = 0;
                write_page_crc = message.start.transfer_crc;
            }
        } else if (state == AWAITING_PAGE_DATA &&
                   message_id == CAN_BOOTLOADER_WRITE_PAGE_DATA) {
            if (message.data.word_index == write_word_index) {
                page_buf[write_word_index] = message.data.word;
                write_word_index++;

                if (write_word_index == write_page_words) {
                    /*
                    Loaded the full page -- validate CRC and start writing to
                    flash
                    */
                    crc = adler32_(write_page_words * sizeof(uint32_t),
                                   page_buf);
                    if (crc == write_page_crc) {
                        /*
                        CRC valid, but only write the page if it has changed
                        */
                        if (has_flash_page_changed_(
                                (uint8_t)write_page_index,
                                page_buf)) {
                            write_flash_page_((uint8_t)write_page_index,
                                              page_buf);
                            message.response.status =
                                (uint8_t)BOOTLOADER_STATUS_OK;
                        } else {
                            message.response.status =
                                (uint8_t)BOOTLOADER_STATUS_UNMODIFIED;
                        }
                    } else {
                        /* CRC invalid */
                        message.response.status =
                            (uint8_t)BOOTLOADER_STATUS_INVALID_CRC;
                    }

                    message.response.page_crc = crc;
                    can_tx_(CAN_BOOTLOADER_WRITE_PAGE_RESPONSE,
                            sizeof(struct can_bootloader_write_page_response_t),
                            message.bytes);

                    state = AWAITING_PAGE_START;
                }
            } else {
                /* Error */
                message.response.page_crc = 0;
                message.response.status =
                    (uint8_t)BOOTLOADER_STATUS_MISSING_DATA;
                can_tx_(CAN_BOOTLOADER_WRITE_PAGE_RESPONSE,
                        sizeof(struct can_bootloader_write_page_response_t),
                        message.bytes);
                state = AWAITING_PAGE_START;
            }
        } else if (message_id == CAN_BOOTLOADER_RESTART) {
            break;
        } else if (state != AWAITING_REQUEST) {
            if (message_id == CAN_BOOTLOADER_REQUEST) {
                /* Got an extra bootloader request, just acknowledege it */
                can_tx_(CAN_BOOTLOADER_ANNOUNCE, 0, message.bytes);
            } else {
                /* Error -- wrong state or message ID */
                message.response.page_crc = 0;
                message.response.status =
                    (uint8_t)BOOTLOADER_STATUS_WRONG_STATE;
                can_tx_(CAN_BOOTLOADER_WRITE_PAGE_RESPONSE,
                        sizeof(struct can_bootloader_write_page_response_t),
                        message.bytes);
            }
        }
    }

    /* NVIC_SystemReset(); */
    __DSB();
    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    for (volatile bool cond = true; cond;);

startup:
    _start();
}

