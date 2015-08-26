/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once


#include "uavcan/transport/transfer_buffer.hpp"
#include "uavcan/transport/crc.hpp"


#define UAVCAN_SOF_BIT 0x80u
#define UAVCAN_EOF_BIT 0x40u
#define UAVCAN_TOGGLE_BIT 0x20u


#define UAVCAN_REQUEST_TIMEOUT 100u


class UAVCANTransferManager {
    uavcan::StaticTransferBuffer<256> tx_buffer_;
    uavcan::StaticTransferBuffer<256> rx_buffer_;

    bool tx_in_progress_;
    uint32_t tx_message_id_;
    size_t tx_offset_;
    uint8_t tx_dest_node_id_;
    uint8_t tx_tail_;
    uavcan::TransferCRC tx_crc_;

    bool rx_in_progress_;
    uint32_t rx_message_id_;
    uint32_t rx_time_;
    uint16_t rx_crc_;
    uint8_t rx_tail_;

    uint8_t node_id_;

    void start_tx_(void) {
        tx_in_progress_ = true;
        tx_tail_ = 0u;
        tx_offset_ = 0u;
        tx_message_id_ = 0u;
        tx_buffer_.reset();
    }

    uint8_t source_node_id_(uint32_t id) const {
        return id & 0x7Fu;
    }

    uint8_t transfer_id_(uint8_t tail) const {
        return tail & 0x1Fu;
    }

    bool is_service_(uint32_t id) const {
        return id & 0x80u;
    }

    bool last_frame_(uint8_t tail) const {
        return tail & UAVCAN_EOF_BIT;
    }

    uint32_t broadcast_message_id_(uint8_t priority, uint16_t dtid) {
        return (priority << 24u) | (dtid << 8u) | node_id_;
    }

    uint32_t response_message_id_(uint32_t request_id) {
        /*
        Zero out "request not response" flag and replace source node ID with
        our own, while setting "service not message" flag and moving the
        original source node ID into the destination node ID field.
        */
        return (request_id & 0xFFFF0000u) | ((request_id & 0x7Fu) << 8u) |
               0x80u | node_id_;
    }

public:
    UAVCANTransferManager(uint8_t node_id) :
        tx_buffer_(),
        rx_buffer_(),
        tx_in_progress_(false),
        tx_message_id_(0u),
        tx_offset_(0u),
        tx_dest_node_id_(0u),
        tx_tail_(0u),
        rx_in_progress_(false),
        rx_message_id_(0u),
        rx_time_(0u),
        rx_crc_(0u),
        rx_tail_(0u),
        node_id_(node_id)
    {}

    bool is_tx_done(void) const {
        return tx_in_progress_ == false;
    }

    bool is_rx_done(void) const {
        return rx_in_progress_ == false && last_frame_(rx_tail_) &&
                rx_message_id_;
    }

    bool is_rx_in_progress(uint32_t in_time) const {
        return rx_in_progress_ && in_time - rx_time_ < UAVCAN_REQUEST_TIMEOUT;
    }

    void receive_frame(
        uint32_t in_time,
        uint32_t id,
        uavcan::TransferCRC base_crc,
        size_t length,
        const uint8_t *data
    ) {
        size_t offset;
        uavcan::TransferCRC message_crc = base_crc;

        /* Abort if the message is too long, or if it'd overflow the buffer */
        if (length > 8u || (int32_t)length > rx_buffer_.getSize() -
                                             rx_buffer_.getMaxWritePos()) {
            return;
        }

        if (rx_in_progress_ && in_time - rx_time_ < UAVCAN_REQUEST_TIMEOUT) {
            if (id == rx_message_id_ &&
                    ((data[length - 1u] ^ rx_tail_) & UAVCAN_TOGGLE_BIT) &&
                    !(data[length - 1u] & UAVCAN_SOF_BIT) &&
                    !((data[length - 1u] ^ rx_tail_) & 0x1Fu)) {
                rx_buffer_.write(rx_buffer_.getMaxWritePos(), data,
                                 length - 1u);
                rx_tail_ = data[length - 1u];

                if (last_frame_(rx_tail_)) {
                    /* Validate the frame CRC; if invalid, clear the flag */
                    message_crc.add(rx_buffer_.getRawPtr(),
                                    rx_buffer_.getMaxWritePos());

                    if (message_crc.get() != rx_crc_) {
                        rx_message_id_ = rx_tail_ = 0u;
                    }

                    rx_in_progress_ = false;
                }
            }
        } else if (data[length - 1u] & UAVCAN_SOF_BIT) {
            rx_tail_ = data[length - 1u];
            if (last_frame_(rx_tail_)) {
                /* Single-frame transfer so done already */
                offset = 0u;
                rx_in_progress_ = false;
            } else {
                rx_crc_ = (uint16_t)(data[0] | (data[1] << 8u));
                offset = 2u;
                rx_in_progress_ = true;
            }
            rx_buffer_.reset();
            if (length - 1u > offset) {
                rx_buffer_.write(rx_buffer_.getMaxWritePos(), &data[offset],
                                 length - offset - 1u);
            }
            rx_message_id_ = id;
            rx_time_ = in_time;
        }
    }

    bool transmit_frame(
        uint32_t& id,
        size_t& length,
        uint8_t *data
    ) {
        bool has_message;
        size_t offset;
        uint16_t crcval;
        uavcan::TransferCRC message_crc;

        if (tx_in_progress_) {
            offset = 0u;
            if (tx_offset_ == 0u && tx_buffer_.getMaxWritePos() > 7u) {
                message_crc = tx_crc_;
                message_crc.add(tx_buffer_.getRawPtr(),
                                tx_buffer_.getMaxWritePos());
                crcval = message_crc.get();
                data[offset++] = (uint8_t)crcval;
                data[offset++] = (uint8_t)(crcval >> 8u);
            }

            id = tx_message_id_;
            length = tx_buffer_.read(tx_offset_, &data[offset], 7u - offset);
            tx_offset_ += length;
            length += offset + 1u;

            data[length - 1u] = tx_tail_;
            tx_tail_ = (uint8_t)((tx_tail_ ^ UAVCAN_TOGGLE_BIT) &
                                 (~UAVCAN_SOF_BIT));

            if (tx_offset_ >= tx_buffer_.getMaxWritePos()) {
                /* Set last frame flag */
                tx_in_progress_ = false;
                data[length - 1u] |= UAVCAN_EOF_BIT;
            }

            has_message = true;
        } else {
            has_message = false;
        }

        return has_message;
    }

    void receive_acknowledge(void) {
        rx_tail_ = rx_message_id_ = 0;
    }

    /* Encoders */
    template<typename T>
    void encode_message(
        uint8_t transfer_id,
        const T& msg
    ) {
        encode_message(transfer_id, msg.DefaultDataTypeID, msg);
    }

    template<typename T>
    void encode_message(
        uint8_t transfer_id,
        uint16_t dtid,
        const T& msg
    ) {
        start_tx_();

        uavcan::BitStream bitstream(tx_buffer_);
        uavcan::ScalarCodec codec(bitstream);
        T::encode(msg, codec);

        tx_tail_ = (uint8_t)((transfer_id & 0x1Fu) | UAVCAN_SOF_BIT);
        tx_message_id_ = broadcast_message_id_(0u, msg.DefaultDataTypeID);
        tx_crc_ = T::getDataTypeSignature().toTransferCRC();
    }

    template<typename T>
    void encode_response(
        const typename T::Response& msg
    ) {
        start_tx_();

        uavcan::BitStream bitstream(tx_buffer_);
        uavcan::ScalarCodec codec(bitstream);
        T::Response::encode(msg, codec);

        tx_tail_ = (uint8_t)((rx_tail_ & 0x1Fu) | UAVCAN_SOF_BIT);
        rx_tail_ = 0u;
        tx_message_id_ = response_message_id_(rx_message_id_);
        tx_crc_ = T::getDataTypeSignature().toTransferCRC();
    }

    /* Decoders */
    template<typename T>
    bool decode(
        T& msg
    ) {
        uavcan::BitStream bitstream(rx_buffer_);
        uavcan::ScalarCodec codec(bitstream);

        return T::decode(msg, codec);
    }
};
