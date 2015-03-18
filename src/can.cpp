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

#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "stm32f30x.h"
#include "hal.h"
#include "can.h"
#include "configuration.h"


#pragma GCC optimize("O3")
static void _bitarray_copy(
    volatile void *dst_org,
    size_t dst_offset,
    const void *src_org,
    size_t src_offset,
    size_t src_len
) {
    static const uint8_t reverse_mask[] =
        { 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };
    static const uint8_t reverse_mask_xor[] =
        { 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00 };

    const uint8_t *src;
    volatile uint8_t *dst;
    uint8_t c;
    size_t src_offset_modulo, dst_offset_modulo, src_len_modulo,
           bit_diff_ls, bit_diff_rs;
    ptrdiff_t byte_len, i;

    if (!src_len) {
        return;
    }

#define PREPARE_FIRST_COPY()                                      \
    do {                                                          \
    if (src_len >= (8u - dst_offset_modulo)) {                    \
        *dst &= reverse_mask[dst_offset_modulo];                  \
        src_len -= 8u - dst_offset_modulo;                        \
    } else {                                                      \
        *dst &= reverse_mask[dst_offset_modulo]                   \
              | reverse_mask_xor[dst_offset_modulo + src_len];    \
         c &= reverse_mask[dst_offset_modulo + src_len];          \
        src_len = 0;                                              \
    } } while (0)

    src = (uint8_t*)src_org + (src_offset >> 3u);
    dst = (uint8_t*)dst_org + (dst_offset >> 3u);

    src_offset_modulo = src_offset & 7u;
    dst_offset_modulo = dst_offset & 7u;

    if (src_offset_modulo == dst_offset_modulo) {
        if (src_offset_modulo) {
            c = reverse_mask_xor[dst_offset_modulo] & *src++;

            PREPARE_FIRST_COPY();
            *dst++ |= c;
        }

        byte_len = src_len >> 3u;
        src_len_modulo = src_len & 7u;

        if (byte_len) {
            for (i = 0; i < byte_len; i++) {
                dst[i] = src[i];
            }
            src += byte_len;
            dst += byte_len;
        }
        if (src_len_modulo) {
            *dst &= reverse_mask_xor[src_len_modulo];
            *dst |= reverse_mask[src_len_modulo] & *src;
        }
    } else {
        /*
         * Begin: Line things up on destination.
         */
        if (src_offset_modulo > dst_offset_modulo) {
            bit_diff_ls = src_offset_modulo - dst_offset_modulo;
            bit_diff_rs = 8u - bit_diff_ls;

            c = (uint8_t)(*src++ << bit_diff_ls);
            c |= (uint8_t)(*src >> bit_diff_rs);
            c &= reverse_mask_xor[dst_offset_modulo];
        } else {
            bit_diff_rs = dst_offset_modulo - src_offset_modulo;
            bit_diff_ls = 8u - bit_diff_rs;

            c = (uint8_t)(*src >> bit_diff_rs & reverse_mask_xor[dst_offset_modulo]);
        }
        PREPARE_FIRST_COPY();
        *dst++ |= c;

        /*
         * Middle: copy with only shifting the source.
         */
        byte_len = src_len >> 3u;

        while (--byte_len >= 0) {
            c = (uint8_t)(*src++ << bit_diff_ls);
            c |= (uint8_t)(*src >> bit_diff_rs);
            *dst++ = c;
        }

        /*
         * End: copy the remaing bits;
         */
        src_len_modulo = src_len & 7u;
        if (src_len_modulo) {
            c = (uint8_t)(*src++ << bit_diff_ls);
            c |= (uint8_t)(*src >> bit_diff_rs);
            c &= reverse_mask[src_len_modulo];

            *dst &= reverse_mask_xor[src_len_modulo];
            *dst |= c;
        }
    }

#undef PREPARE_FIRST_COPY
}


static uint64_t _get_data_type_signature(uint16_t data_type_id) {
    uint64_t signature;
    switch (data_type_id) {
        case UAVCAN_RAWCOMMAND:
            signature = UAVCAN_RAWCOMMAND_SIGNATURE;
            break;
        case UAVCAN_RPMCOMMAND:
            signature = UAVCAN_RPMCOMMAND_SIGNATURE;
            break;
        case UAVCAN_ESCSTATUS:
            signature = UAVCAN_ESCSTATUS_SIGNATURE;
            break;
        case UAVCAN_GETNODEINFO:
            signature = UAVCAN_GETNODEINFO_SIGNATURE;
            break;
        case UAVCAN_GETSET:
            signature = UAVCAN_GETSET_SIGNATURE;
            break;
        default:
            signature = 0;
            break;
    }
    return signature;
}


void UAVCANServer::serialize_node_status(
    UAVCANMessage& out_message,
    uint8_t transfer_id,
    uint32_t uptime_s,
    enum uavcan_nodestatus_t status
) {
    /*
    550.NodeStatus

    uint28 uptime_sec
    uint4 status_code
    */
    out_message.set_transfer_id(transfer_id);
    out_message.set_last_frame(true);
    out_message.set_frame_index(0);
    out_message.set_source_node_id(config_local_node_id_);
    out_message.set_transfer_type(UAVCAN_TRANSFER_MESSAGE_BROADCAST);
    out_message.set_data_type_id(UAVCAN_NODESTATUS);
    out_message.set_length(4);
    out_message[0] = (uint8_t)((uptime_s >> 0u) & 0xFFu);
    out_message[1] = (uint8_t)((uptime_s >> 8u) & 0xFFu);
    out_message[2] = (uint8_t)((uptime_s >> 16u) & 0xFFu);
    out_message[3] = (uint8_t)
        ((((uptime_s >> 24u) & 0x0Fu) << 4u) | (status & 0x0Fu));
}


bool UAVCANServer::parse_restartnode_request() {
    /*
    560.RestartNode

    uint40 magic_number = 0xACCE551B1E
    ---
    bool ok
    */
    uint64_t magic_number;

    _bitarray_copy(&magic_number, 0, rx_transfer_bytes_, 0, 40u);

    if (magic_number == 0xACCE551B1Eull) {
        return true;
    } else {
        return false;
    }
}


void UAVCANServer::serialize_restartnode_reply(bool ok) {
    tx_transfer_transfer_type_transfer_id_ = (uint8_t)
        ((UAVCAN_TRANSFER_SERVICE_RESPONSE << 4u) |
         rx_transfer_.get_transfer_id());
    tx_transfer_dest_node_id_ = rx_transfer_.get_source_node_id();
    tx_transfer_data_type_id_ = UAVCAN_RESTARTNODE;
    tx_transfer_bytes_[0] = (uint8_t)(ok ? 0x80u : 0);
    tx_transfer_length_ = 1;
    tx_transfer_index_ = 0;
    tx_transfer_frame_index_ = 0;
    tx_transfer_done_ = false;
}


bool UAVCANServer::parse_saveerase_request(
    enum uavcan_saveerase_opcode_t &out_opcode
) {
    /*
    598.SaveErase

    uint2 opcode
    ---
    bool ok
    */
    out_opcode = (enum uavcan_saveerase_opcode_t)(rx_transfer_bytes_[0] >> 6u);
    return true;
}


void UAVCANServer::serialize_saveerase_reply(bool ok) {
    /*
    598.SaveErase

    uint2 opcode
    ---
    bool ok
    */
    tx_transfer_transfer_type_transfer_id_ = (uint8_t)
        ((UAVCAN_TRANSFER_SERVICE_RESPONSE << 4u) |
         rx_transfer_.get_transfer_id());
    tx_transfer_dest_node_id_ = rx_transfer_.get_source_node_id();
    tx_transfer_data_type_id_ = UAVCAN_SAVEERASE;
    tx_transfer_bytes_[0] = (uint8_t)(ok ? 0x80u : 0);
    tx_transfer_length_ = 1u;
    tx_transfer_index_ = 0;
    tx_transfer_frame_index_ = 0;
    tx_transfer_done_ = false;
}


bool UAVCANServer::parse_getset_request(
    float& out_value,
    uint8_t& out_param_index,
    char *out_param_name
) {
    /*
    599.GetSet

    Value value
    uint8 index
    uint8[<=40] name
    ---
    Value value
    Value default_value
    Value max_value
    Value min_value
    uint8[<=40] name

    Value
    bool[<=1] value_bool
    int64[<=1] value_int
    float32[<=1] value_float
    */
    int64_t bits;
    uint32_t temp;
    size_t bit_index, transfer_bits, name_bytes;

    bit_index = 0;
    transfer_bits = rx_transfer_length_ * 8u;
    out_value = std::numeric_limits<float>::signaling_NaN();

    /* Extract the value */
    bits = 0;
    _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 1u);
    bit_index++;
    if (bits) {
        /* Boolean value */
        bits = 0;
        _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 1u);
        bit_index++;

        if (std::isnan(out_value)) {
            out_value = bits ? 1.0f : 0.0f;
        }
    }

    bits = 0;
    _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 1u);
    bit_index++;
    if (bits) {
        /* Int64 value -- only support writing the low word */
        bits = 0;
        _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 64u);
        bit_index += 64u;

        if (std::isnan(out_value)) {
            temp = (uint32_t)bits;
            out_value = (float)temp;
        }
    }

    bits = 0;
    _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 1u);
    bit_index++;
    if (bits) {
        bits = 0;
        _bitarray_copy(&bits, 0, rx_transfer_bytes_, bit_index, 32u);
        bit_index += 32u;

        if (std::isnan(out_value)) {
            memcpy(&out_value, &bits, sizeof(float));
        }
    }

    /* Extract the parameter index */
    _bitarray_copy(&out_param_index, 0, rx_transfer_bytes_, bit_index, 8u);
    bit_index += 8u;

    /* Extract the name -- uses tail array optimization */
    name_bytes = (transfer_bits - bit_index) >> 3u;
    if (name_bytes < 27u) {
        _bitarray_copy(out_param_name, 0, rx_transfer_bytes_, bit_index,
                       transfer_bits - bit_index);
        out_param_name[name_bytes] = 0;
        return true;
    } else {
        return false;
    }
}


void UAVCANServer::serialize_getset_reply(const struct param_t &in_param) {
    /*
    599.GetSet

    Value value
    uint8 index
    uint8[<=40] name
    ---
    Value value
    Value default_value
    Value max_value
    Value min_value
    uint8[<=40] name

    Value
    bool[<=1] value_bool
    int64[<=1] value_int
    float32[<=1] value_float
    */
    size_t bit_index, pad_bits, length, name_bits;
    uint8_t value_type;

    bit_index = 0;

#define SERIALIZE_FLOAT_VALUE(x) {                                           \
        if (!std::isnan(x)) {                                                \
            value_type = 0x20u;                                              \
            _bitarray_copy(tx_transfer_bytes_, bit_index, &value_type, 0, 3u);\
            bit_index += 3u;                                                 \
            _bitarray_copy(tx_transfer_bytes_, bit_index, &x, 0, 32u);       \
            bit_index += 32u;                                                 \
        } else {                                                             \
            value_type = 0;                                                  \
            _bitarray_copy(tx_transfer_bytes_, bit_index, &value_type, 0, 3u);\
            bit_index += 3u;                                                 \
        }                                                                    \
    }

    SERIALIZE_FLOAT_VALUE(in_param.value);
    SERIALIZE_FLOAT_VALUE(in_param.default_value);
    SERIALIZE_FLOAT_VALUE(in_param.max_value);
    SERIALIZE_FLOAT_VALUE(in_param.min_value);

#undef SERIALIZE_FLOAT_VALUE

    name_bits = (strnlen(in_param.name, 27u) << 3u);
    _bitarray_copy(tx_transfer_bytes_, bit_index, in_param.name, 0,
                   name_bits);
    bit_index += name_bits;

    /* Length is rounded up to the next byte */
    value_type = 0;
    length = (bit_index + 7u) >> 3u;
    pad_bits = (length << 3u) - bit_index;
    _bitarray_copy(tx_transfer_bytes_, bit_index, &value_type, 0, pad_bits);

    tx_transfer_transfer_type_transfer_id_ = (uint8_t)
        ((UAVCAN_TRANSFER_SERVICE_RESPONSE << 4u) |
         rx_transfer_.get_transfer_id());
    tx_transfer_dest_node_id_ = rx_transfer_.get_source_node_id();
    tx_transfer_data_type_id_ = UAVCAN_GETSET;
    tx_transfer_length_ = length;
    tx_transfer_index_ = 0;
    tx_transfer_frame_index_ = 0;
    tx_transfer_done_ = false;
}


float UAVCANServer::get_esccommand_value() {
    /*
    260.RawCommand

    int14[<=20] cmd

    261.RPMCommand

    int18[<=20] rpm
    */
    uint8_t field_width;
    uint32_t value, bit_index;
    float result;

    result = std::numeric_limits<float>::signaling_NaN();

    if (rx_transfer_.get_data_type_id() == UAVCAN_RAWCOMMAND) {
        field_width = 14u;
    } else if (rx_transfer_.get_data_type_id() == UAVCAN_RPMCOMMAND) {
        field_width = 18u;
    } else {
        return result;
    }

    value = 0;

    bit_index = config_esc_index_ * field_width;

    if (bit_index + field_width <= rx_transfer_length_ * 8u) {
        _bitarray_copy(&value, 0, rx_transfer_bytes_, bit_index, field_width);
        if (value >= (1u << (field_width - 1u))) {
            /*
            The value is negative, so convert to float exluding the sign bit
            and
            */
            result = -(float)((1u << field_width) - value);
        } else {
            result = (float)value;
        }
    }

    return result;
}


void UAVCANServer::serialize_esc_status(
    const struct uavcan_esc_state_t& state
) {
    /*
    601.Status

    uint32 error_count
    float16 voltage
    float16 current
    float16 temperature
    int18 rpm
    uint7 power_rating_pct
    uint5 esc_index
    */

    size_t bit_index, pad_bits, length;
    float16_t temp;
    int32_t itemp;
    uint8_t utemp;

    memset(status_transfer_bytes_, 0, sizeof(status_transfer_bytes_));

    bit_index = 32u;

    temp = float16(state.vbus_v);
    _bitarray_copy(status_transfer_bytes_, bit_index, &temp, 0, 16u);
    bit_index += 16u;

    temp = float16(state.ibus_a);
    _bitarray_copy(status_transfer_bytes_, bit_index, &temp, 0, 16u);
    bit_index += 16u;

    temp = float16(state.temperature_degc + 273.15f);
    _bitarray_copy(status_transfer_bytes_, bit_index, &temp, 0, 16u);
    bit_index += 16u;

    itemp = (int32_t)state.speed_rpm;
    _bitarray_copy(status_transfer_bytes_, bit_index, &itemp, 0, 18u);
    bit_index += 18u;

    utemp = (uint8_t)state.power_pct;
    _bitarray_copy(status_transfer_bytes_, bit_index, &utemp, 0, 7u);
    bit_index += 7u;

    utemp = (uint8_t)config_esc_index_;
    _bitarray_copy(status_transfer_bytes_, bit_index, &utemp, 0, 5u);
    bit_index += 5u;

    /* Add padding if required and set the lengths */
    utemp = 0;
    length = (bit_index + 7u) >> 3u;
    pad_bits = (length << 3u) - bit_index;
    _bitarray_copy(status_transfer_bytes_, bit_index, &utemp, 0, pad_bits);

    status_transfer_length_ = length;
    status_transfer_index_ = 0;
    status_transfer_frame_index_ = 0;
}


void UAVCANServer::tick(const struct uavcan_esc_state_t& state) {
    if (config_esc_status_interval_ > 0 &&
            status_transfer_length_ == status_transfer_index_ &&
            uptime_ms_ - last_esc_status_sent_time_ms_ >
            config_esc_status_interval_) {
        last_esc_status_sent_time_ms_ = uptime_ms_;
        status_transfer_length_ = status_transfer_index_ =
            status_transfer_frame_index_ = 0;

        serialize_esc_status(state);
    }

    if (received_partial_frame_) {
        last_partial_frame_received_time_ms_ = uptime_ms_;
        received_partial_frame_ = false;
    }

    if (uptime_ms_ - last_partial_frame_received_time_ms_ >
            UAVCAN_TRANSFER_TIMEOUT_INTERVAL_MS && !rx_transfer_reset_) {
        rx_transfer_reset_ = true;
        last_partial_frame_received_time_ms_ = uptime_ms_;
    }

    uptime_ms_++;
}


bool UAVCANServer::process_tx(UAVCANMessage& out_message) {
    uint16_t crc;
    uint8_t i, transfer_type;

    if (tx_transfer_index_ < tx_transfer_length_) {
        i = 0;
        transfer_type = (tx_transfer_transfer_type_transfer_id_ & 0xF0u) >> 4u;

        out_message.set_transfer_id(tx_transfer_transfer_type_transfer_id_ & 7u);
        out_message.set_source_node_id(config_local_node_id_);
        out_message.set_transfer_type(transfer_type);
        out_message.set_data_type_id(tx_transfer_data_type_id_);

        /* Non-broadcast transfers only permit 7 bytes per frame */
        if (transfer_type != UAVCAN_TRANSFER_MESSAGE_BROADCAST) {
            out_message[i++] = tx_transfer_dest_node_id_;
        }

        /*
        First frame of a multi-frame transfer has 2 fewer bytes available
        */
        if (tx_transfer_index_ == 0 && tx_transfer_length_ > 8u - i) {
            /* Generate CRC */
            crc = compute_crc(
                _get_data_type_signature(tx_transfer_data_type_id_),
                tx_transfer_bytes_,
                tx_transfer_length_
            );
            out_message[i++] = (uint8_t)(crc & 0xFFu);
            out_message[i++] = (uint8_t)((crc >> 8u) & 0xFFu);
        }

        for (; i < 8 && tx_transfer_index_ < tx_transfer_length_;
                i++, tx_transfer_index_++) {
            out_message[i] = tx_transfer_bytes_[tx_transfer_index_];
        }

        out_message.set_length(i);
        out_message.set_frame_index((uint8_t)(tx_transfer_frame_index_++));
        if (tx_transfer_index_ == tx_transfer_length_) {
            out_message.set_last_frame(true);
            tx_transfer_done_ = true;
        }

        return true;
    } else if (status_transfer_index_ < status_transfer_length_) {
        i = 0;
        out_message.set_transfer_id((broadcast_transfer_ids_[1]++) & 7u);
        out_message.set_source_node_id(config_local_node_id_);
        out_message.set_transfer_type(UAVCAN_TRANSFER_MESSAGE_BROADCAST);
        out_message.set_data_type_id(UAVCAN_ESCSTATUS);

        if (status_transfer_index_ == 0) {
            /* Generate CRC */
            crc = compute_crc(
                UAVCAN_ESCSTATUS_SIGNATURE,
                status_transfer_bytes_,
                status_transfer_length_
            );
            out_message[i++] = (uint8_t)(crc & 0xFFu);
            out_message[i++] = (uint8_t)((crc >> 8u) & 0xFFu);
        }

        for (; i < 8 && status_transfer_index_ < status_transfer_length_;
                i++, status_transfer_index_++) {
            out_message[i] = status_transfer_bytes_[status_transfer_index_];
        }

        out_message.set_length(i);
        if (status_transfer_index_ == status_transfer_index_) {
            out_message.set_last_frame(true);
        }
        out_message.set_frame_index((uint8_t)(status_transfer_frame_index_++));

        return true;
    } else if (uptime_ms_ - last_node_status_sent_time_ms_ >
                   UAVCAN_NODESTATUS_INTERVAL_MS) {
        /* Send node status every 512 ms */
        serialize_node_status(out_message, (broadcast_transfer_ids_[0]++) & 7u,
                              uptime_ms_ / 1000,
                              UAVCAN_NODESTATUS_OK);
        last_node_status_sent_time_ms_ = uptime_ms_;
        return true;
    } else {
        return false;
    }
}


void UAVCANServer::process_rx(const UAVCANMessage& in_message) {
    uint64_t signature;
    size_t i;
    uint16_t data_type_id;
    uint8_t dest_node_id;

    data_type_id = in_message.get_data_type_id();
    dest_node_id = in_message.get_dest_node_id();

    if (!tx_transfer_done_) {
        /* Don't process anything until the last transmission is done */
    } else if (rx_transfer_length_) {
        /*
        Currently processing a message -- ignore any packets that aren't part
        of this transfer.
        */
        if (data_type_id == rx_transfer_.get_data_type_id() &&
                dest_node_id == rx_transfer_.get_dest_node_id() &&
                in_message.get_transfer_id() == rx_transfer_.get_transfer_id() &&
                in_message.get_source_node_id() == rx_transfer_.get_source_node_id() &&
                in_message.get_frame_index() == rx_transfer_frame_index_ + 1) {
            received_partial_frame_ = true;

            /* Skip the destination node ID */
            i = in_message.get_transfer_type() ==
                    UAVCAN_TRANSFER_MESSAGE_BROADCAST ? 0 : 1;

            /* Add the message payload */
            for (; i < in_message.get_length(); i++) {
                rx_transfer_bytes_[rx_transfer_length_++] = in_message[i];
            }
            rx_transfer_frame_index_++;
            rx_transfer_reset_ = false;

            /*
            If the message has the last_frame flag set, validate the CRC and
            process the transfer.
            */
            if (in_message.get_last_frame()) {
                signature = _get_data_type_signature(data_type_id);
                if (rx_transfer_frame_index_ == 1 ||
                        compute_crc(signature, rx_transfer_bytes_,
                                    rx_transfer_length_) == rx_transfer_crc_) {
                    process_transfer();
                }
                rx_transfer_length_ = 0;
                rx_transfer_frame_index_ = 0;
                rx_transfer_reset_ = true;
            }
        } else if (rx_transfer_reset_) {
            /*
            Transfer timed out, so reset it.
            */
            rx_transfer_length_ = 0;
            rx_transfer_frame_index_ = 0;
        } else {
            /* ignore */
        }
    } else if (((data_type_id == UAVCAN_RAWCOMMAND ||
                 data_type_id == UAVCAN_RPMCOMMAND) && dest_node_id == 0xFFu) ||
               ((data_type_id == UAVCAN_RESTARTNODE ||
                 data_type_id == UAVCAN_SAVEERASE ||
                 data_type_id == UAVCAN_GETSET ||
                 data_type_id == UAVCAN_GETNODEINFO) && dest_node_id == config_local_node_id_)) {
        received_partial_frame_ = true;

        /*
        Not currently processing a message -- wait for a relevant packet to
        come along and then start the transfer.
        */
        rx_transfer_ = in_message;
        rx_transfer_length_ = 0;
        rx_transfer_frame_index_ = 0;
        rx_transfer_reset_ = false;

        /*
        One byte for destination node ID in non-broadcast frames, plus two
        CRC bytes for the first frame of a multi-frame transfer
        */
        i = (in_message.get_transfer_type() ==
                UAVCAN_TRANSFER_MESSAGE_BROADCAST ? 0u : 1u);

        if (!in_message.get_last_frame()) {
            rx_transfer_crc_ = in_message[i++];
            rx_transfer_crc_ += in_message[i++] << 8u;
        }

        for (; i < in_message.get_length(); i++) {
            rx_transfer_bytes_[rx_transfer_length_++] = in_message[i];
        }

        /*
        If the message has the last_frame flag set, process the transfer
        */
        if (in_message.get_last_frame()) {
            process_transfer();
            rx_transfer_length_ = 0;
            rx_transfer_frame_index_ = 0;
            rx_transfer_reset_ = true;
        }
    } else {
        /* Not a packet we care about -- ignore it */
    }
}


void UAVCANServer::process_transfer() {
    float result;
    enum uavcan_saveerase_opcode_t saveerase_opcode;
    struct param_t param;
    enum uavcan_data_type_id_t data_type_id;
    size_t i;
    bool valid;
    uint8_t index;
    char name[27];

    data_type_id = (enum uavcan_data_type_id_t)rx_transfer_.get_data_type_id();

    switch (data_type_id) {
        case UAVCAN_RAWCOMMAND:
        case UAVCAN_RPMCOMMAND:
            result = get_esccommand_value();
            if (!std::isnan(result) && esc_command_cb_) {
                esc_command_cb_(data_type_id, result);
            }
            break;
        case UAVCAN_GETNODEINFO:
            // serialize_nodeinfo_reply();
            break;
        case UAVCAN_RESTARTNODE:
            valid = parse_restartnode_request();
            if (valid && restart_request_cb_) {
                valid = restart_request_cb_();
            }
            serialize_restartnode_reply(valid);
            break;
        case UAVCAN_SAVEERASE:
            valid = parse_saveerase_request(saveerase_opcode);
            if (valid) {
                if (saveerase_opcode == UAVCAN_SAVEERASE_OPCODE_ERASE) {
                    /* Reset parameters to their default values */
                    for (i = 0; i < NUM_PARAMS; i++) {
                        /* Don't reset the node ID */
                        if (i == PARAM_UAVCAN_NODE_ID) {
                            continue;
                        }

                        configuration_->get_param_by_index(param, (uint8_t)i);
                        configuration_->set_param_value_by_index(
                            (uint8_t)i, param.default_value);
                    }
                }

                /* Save the current parameter set if permissible */
                if (flash_save_request_cb_) {
                    valid = flash_save_request_cb_();
                }
            }
            serialize_saveerase_reply(valid);
            break;
        case UAVCAN_GETSET:
            valid = parse_getset_request(result, index, name);
            if (valid) {
                if (name[0]) {
                    configuration_->set_param_value_by_name(name, result);
                    valid = configuration_->get_param_by_name(param, name);
                } else {
                    configuration_->set_param_value_by_index(index, result);
                    valid = configuration_->get_param_by_index(param, index);
                }
            }

            if (!valid) {
                param.value = std::numeric_limits<float>::signaling_NaN();
                param.default_value = std::numeric_limits<float>::signaling_NaN();
                param.min_value = std::numeric_limits<float>::signaling_NaN();
                param.max_value = std::numeric_limits<float>::signaling_NaN();
                param.index = 0;
                param.name[0] = 0;
            }

            serialize_getset_reply(param);
            break;
        default:
            break;
    }
}


#pragma GCC optimize("O0")
uint16_t UAVCANServer::compute_crc(
    uint64_t data_type_signature,
    const volatile uint8_t* payload,
    size_t length
) {
    static uint16_t crc[256] = {
        0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
        0x8108u, 0x9129u, 0xA14Au, 0xB16Bu, 0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
        0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52B5u, 0x4294u, 0x72F7u, 0x62D6u,
        0x9339u, 0x8318u, 0xB37Bu, 0xA35Au, 0xD3BDu, 0xC39Cu, 0xF3FFu, 0xE3DEu,
        0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64E6u, 0x74C7u, 0x44A4u, 0x5485u,
        0xA56Au, 0xB54Bu, 0x8528u, 0x9509u, 0xE5EEu, 0xF5CFu, 0xC5ACu, 0xD58Du,
        0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76D7u, 0x66F6u, 0x5695u, 0x46B4u,
        0xB75Bu, 0xA77Au, 0x9719u, 0x8738u, 0xF7DFu, 0xE7FEu, 0xD79Du, 0xC7BCu,
        0x48C4u, 0x58E5u, 0x6886u, 0x78A7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
        0xC9CCu, 0xD9EDu, 0xE98Eu, 0xF9AFu, 0x8948u, 0x9969u, 0xA90Au, 0xB92Bu,
        0x5AF5u, 0x4AD4u, 0x7AB7u, 0x6A96u, 0x1A71u, 0x0A50u, 0x3A33u, 0x2A12u,
        0xDBFDu, 0xCBDCu, 0xFBBFu, 0xEB9Eu, 0x9B79u, 0x8B58u, 0xBB3Bu, 0xAB1Au,
        0x6CA6u, 0x7C87u, 0x4CE4u, 0x5CC5u, 0x2C22u, 0x3C03u, 0x0C60u, 0x1C41u,
        0xEDAEu, 0xFD8Fu, 0xCDECu, 0xDDCDu, 0xAD2Au, 0xBD0Bu, 0x8D68u, 0x9D49u,
        0x7E97u, 0x6EB6u, 0x5ED5u, 0x4EF4u, 0x3E13u, 0x2E32u, 0x1E51u, 0x0E70u,
        0xFF9Fu, 0xEFBEu, 0xDFDDu, 0xCFFCu, 0xBF1Bu, 0xAF3Au, 0x9F59u, 0x8F78u,
        0x9188u, 0x81A9u, 0xB1CAu, 0xA1EBu, 0xD10Cu, 0xC12Du, 0xF14Eu, 0xE16Fu,
        0x1080u, 0x00A1u, 0x30C2u, 0x20E3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
        0x83B9u, 0x9398u, 0xA3FBu, 0xB3DAu, 0xC33Du, 0xD31Cu, 0xE37Fu, 0xF35Eu,
        0x02B1u, 0x1290u, 0x22F3u, 0x32D2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
        0xB5EAu, 0xA5CBu, 0x95A8u, 0x8589u, 0xF56Eu, 0xE54Fu, 0xD52Cu, 0xC50Du,
        0x34E2u, 0x24C3u, 0x14A0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
        0xA7DBu, 0xB7FAu, 0x8799u, 0x97B8u, 0xE75Fu, 0xF77Eu, 0xC71Du, 0xD73Cu,
        0x26D3u, 0x36F2u, 0x0691u, 0x16B0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
        0xD94Cu, 0xC96Du, 0xF90Eu, 0xE92Fu, 0x99C8u, 0x89E9u, 0xB98Au, 0xA9ABu,
        0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18C0u, 0x08E1u, 0x3882u, 0x28A3u,
        0xCB7Du, 0xDB5Cu, 0xEB3Fu, 0xFB1Eu, 0x8BF9u, 0x9BD8u, 0xABBBu, 0xBB9Au,
        0x4A75u, 0x5A54u, 0x6A37u, 0x7A16u, 0x0AF1u, 0x1AD0u, 0x2AB3u, 0x3A92u,
        0xFD2Eu, 0xED0Fu, 0xDD6Cu, 0xCD4Du, 0xBDAAu, 0xAD8Bu, 0x9DE8u, 0x8DC9u,
        0x7C26u, 0x6C07u, 0x5C64u, 0x4C45u, 0x3CA2u, 0x2C83u, 0x1CE0u, 0x0CC1u,
        0xEF1Fu, 0xFF3Eu, 0xCF5Du, 0xDF7Cu, 0xAF9Bu, 0xBFBAu, 0x8FD9u, 0x9FF8u,
        0x6E17u, 0x7E36u, 0x4E55u, 0x5E74u, 0x2E93u, 0x3EB2u, 0x0ED1u, 0x1EF0u
    };

    uint16_t result, i;
    uint8_t signature_bytes[8];

    memcpy(signature_bytes, &data_type_signature, 8);

    result = 0xFFFFu;

#define CRC_STEP(r, x) (uint16_t)((((r) << 8u) ^ crc[((r) >> 8u) ^ (x)]) & 0xFFFFu)
    for (i = 0; i < 8; i++) {
        result = CRC_STEP(result, signature_bytes[i]);
    }
    for (i = 0; i < length; i++) {
        result = CRC_STEP(result, payload[i]);
    }
#undef CRC_STEP

    return result;
}
