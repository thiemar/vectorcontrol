/*
Copyright (c) 2014 - 2015 Thiemar Pty Ltd

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


#include "uavcan/transport/transfer_buffer.hpp"
#include "uavcan/transport/crc.hpp"
#include "uavcan/protocol/param/ExecuteOpcode.hpp"
#include "uavcan/protocol/param/GetSet.hpp"
#include "uavcan/protocol/file/BeginFirmwareUpdate.hpp"
#include "uavcan/protocol/GetNodeInfo.hpp"
#include "uavcan/protocol/NodeStatus.hpp"
#include "uavcan/protocol/RestartNode.hpp"
#include "uavcan/equipment/esc/RawCommand.hpp"
#include "uavcan/equipment/esc/RPMCommand.hpp"
#include "uavcan/equipment/esc/Status.hpp"
#include "thiemar/equipment/esc/Status.hpp"


enum uavcan_dtid_filter_id_t {
    UAVCAN_EQUIPMENT_ESC_RAWCOMMAND = 0u,
    UAVCAN_EQUIPMENT_ESC_RPMCOMMAND,
    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE,
    UAVCAN_PROTOCOL_PARAM_GETSET,
    UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE,
    UAVCAN_PROTOCOL_GETNODEINFO,
    UAVCAN_PROTOCOL_RESTARTNODE
};


enum uavcan_transfertype_t {
    SERVICE_RESPONSE = 0u,
    SERVICE_REQUEST = 1u,
    MESSAGE_BROADCAST = 2u,
    MESSAGE_UNICAST = 3u
};


#define UAVCAN_REQUEST_TIMEOUT 100u


class UAVCANTransferManager {
    uavcan::StaticTransferBuffer<256> tx_buffer_;
    uavcan::StaticTransferBuffer<256> rx_buffer_;
    uavcan::BitStream tx_bitstream_;
    uavcan::BitStream rx_bitstream_;
    uavcan::ScalarCodec tx_codec_;
    uavcan::ScalarCodec rx_codec_;

    bool tx_in_progress_;
    uint32_t tx_message_id_;
    size_t tx_offset_;
    uint8_t tx_dest_node_id_;

    bool rx_in_progress_;
    uint32_t rx_message_id_;
    uint32_t rx_time_;
    uint16_t rx_crc_;

    uint8_t node_id_;

    void start_tx_(void) {
        tx_in_progress_ = true;
    }

    uint8_t source_node_id_(uint32_t id) {
        return (id >> 10u) & 0x7Fu;
    }

    uint8_t frame_index_(uint32_t id) {
        return (id >> 4u) & 0x3Fu;
    }

    uint8_t transfer_id_(uint32_t id) {
        return id & 0x7u;
    }

    enum uavcan_transfertype_t transfer_type_(uint32_t id) const {
        return (enum uavcan_transfertype_t)((id >> 17u) & 0x3u);
    }

    bool last_frame_(uint32_t id) const {
        return id & 0x8u;
    }

    uint32_t broadcast_message_id_(uint8_t transfer_id, uint16_t dtid) {
        return (dtid << 19u) | (MESSAGE_BROADCAST << 17u) |
               (transfer_id & 0x7u);
    }

    uint32_t response_message_id_(uint32_t request_id) {
        /*
        Zero out transfer type (for response), frame index and last frame
        flag, while replacing source node ID with our own.
        */
        return (request_id & ~0x7FFF8u) | (node_id_ << 10u);
    }

public:
    UAVCANTransferManager(uint8_t node_id) :
        tx_buffer_(),
        rx_buffer_(),
        tx_bitstream_(tx_buffer_),
        rx_bitstream_(rx_buffer_),
        tx_codec_(tx_bitstream_),
        rx_codec_(rx_bitstream_),
        tx_in_progress_(false),
        tx_message_id_(0u),
        tx_offset_(0u),
        tx_dest_node_id_(0u),
        rx_in_progress_(false),
        rx_message_id_(0u),
        rx_time_(0u),
        rx_crc_(0u),
        node_id_(node_id)
    {}

    bool is_tx_done(void) const {
        return tx_in_progress_ == false;
    }

    bool is_rx_done(void) const {
        return last_frame_(rx_message_id_);
    }

    void receive_frame(
        uint32_t in_time,
        uint32_t id,
        size_t length,
        const uint8_t *data
    ) {
        size_t offset;
        uavcan::TransferCRC message_crc;

        if (rx_in_progress_ && in_time - rx_time_ < UAVCAN_REQUEST_TIMEOUT) {
            if (source_node_id_(id) == source_node_id_(rx_message_id_) &&
                    frame_index_(id) == frame_index_(rx_message_id_) + 1u &&
                    transfer_id_(id) == transfer_id_(rx_message_id_)) {
                offset = transfer_type_(id) == MESSAGE_BROADCAST ? 1u : 0u;
                if (length > offset) {
                    rx_buffer_.write(rx_buffer_.getMaxWritePos(),
                                     &data[offset], length - offset);
                }
                rx_message_id_ = id;

                if (last_frame_(id)) {
                    /* Validate the frame CRC; if invalid, clear the flag */
                    message_crc.add(rx_buffer_.getRawPtr(),
                                    rx_buffer_.getMaxWritePos());

                    if (message_crc.get() != rx_crc_) {
                        rx_message_id_ = 0u;
                    }

                    rx_in_progress_ = false;
                }
            }
        } else if (frame_index_(id) == 0u) {
            offset = transfer_type_(id) == MESSAGE_BROADCAST ? 1u : 0u;
            if (last_frame_(id)) {
                /* Single-frame transfer so done already */
                rx_in_progress_ = false;
            } else {
                rx_crc_ = (uint16_t)(data[0] | (data[1] << 8u));
                offset += 2u;
                rx_in_progress_ = true;
            }
            rx_buffer_.reset();
            rx_bitstream_.reset();
            if (length > offset) {
                rx_buffer_.write(rx_buffer_.getMaxWritePos(), &data[offset],
                                 length - offset);
            }
            rx_message_id_ = id;
        }
    }

    bool transmit_frame(
        uint32_t& id,
        size_t& length,
        uint8_t *data
    ) {
        bool has_message;
        size_t offset;
        uavcan::TransferCRC message_crc;

        if (tx_in_progress_) {
            if (transfer_type_(tx_message_id_) == MESSAGE_BROADCAST) {
                offset = 0u;
            } else {
                offset = 1u;
                data[0] = tx_dest_node_id_;
            }

            if (frame_index_(tx_message_id_) == 0u &&
                    tx_buffer_.getMaxWritePos() > 8u - offset) {
                data[offset++] = (uint8_t)message_crc.get();
                data[offset++] = (uint8_t)(message_crc.get() >> 8u);
            }

            id = tx_message_id_;
            length = tx_buffer_.read(tx_offset_, data, 8u - offset);
            tx_offset_ += length;
            length += offset;

            if (tx_offset_ >= tx_buffer_.getMaxWritePos()) {
                /* Set last frame flag */
                id |= 8u;
                tx_in_progress_ = false;
                tx_offset_ = 0u;
                tx_message_id_ = 0u;
                tx_buffer_.reset();
                tx_bitstream_.reset();
            } else {
                /* Increment the frame index */
                tx_message_id_ += 0x10u;
            }

            has_message = true;
        } else {
            has_message = false;
        }

        return has_message;
    }

    /* Encoders */

    void encode_nodestatus(
        uint8_t transfer_id,
        const uavcan::protocol::NodeStatus& msg
    ) {
        start_tx_();
        uavcan::protocol::NodeStatus::encode(msg, tx_codec_);
        tx_message_id_ = broadcast_message_id_(
            transfer_id, msg.DefaultDataTypeID);
    }

    void encode_esc_status(
        uint8_t transfer_id,
        const uavcan::equipment::esc::Status& msg
    ) {
        start_tx_();
        uavcan::equipment::esc::Status::encode(msg, tx_codec_);
        tx_message_id_ = broadcast_message_id_(
            transfer_id, msg.DefaultDataTypeID);
    }

    void encode_custom_status(
        uint8_t transfer_id,
        uint16_t dtid,
        const thiemar::equipment::esc::Status& msg
    ) {
        start_tx_();
        thiemar::equipment::esc::Status::encode(msg, tx_codec_);
        tx_message_id_ = broadcast_message_id_(transfer_id, dtid);
    }

    void encode_executeopcode_response(
        const uavcan::protocol::param::ExecuteOpcode::Response& msg
    ) {
        start_tx_();
        uavcan::protocol::param::ExecuteOpcode::Response::encode(
            msg, tx_codec_);
        tx_message_id_ = response_message_id_(rx_message_id_);
    }

    void encode_getset_response(
        const uavcan::protocol::param::GetSet::Response& msg
    ) {
        start_tx_();
        uavcan::protocol::param::GetSet::Response::encode(msg, tx_codec_);
        tx_message_id_ = response_message_id_(rx_message_id_);
    }

    void encode_beginfirmwareupdate_response(
        const uavcan::protocol::file::BeginFirmwareUpdate::Response& msg
    ) {
        start_tx_();
        uavcan::protocol::file::BeginFirmwareUpdate::Response::encode(
            msg, tx_codec_);
        tx_message_id_ = response_message_id_(rx_message_id_);
    }

    void encode_getnodeinfo_response(
        const uavcan::protocol::GetNodeInfo::Response& msg
    ) {
        start_tx_();
        uavcan::protocol::GetNodeInfo::Response::encode(msg, tx_codec_);
        tx_message_id_ = response_message_id_(rx_message_id_);
    }

    void encode_restartnode_response(
        const uavcan::protocol::RestartNode::Response& msg
    ) {
        start_tx_();
        uavcan::protocol::RestartNode::Response::encode(msg, tx_codec_);
        tx_message_id_ = response_message_id_(rx_message_id_);
    }

    /* Decoders */

    bool decode_esc_rawcommand(
        uavcan::equipment::esc::RawCommand& msg
    ) {
        return uavcan::equipment::esc::RawCommand::decode(msg, rx_codec_);
    }

    bool decode_esc_rpmcommand(
        uavcan::equipment::esc::RPMCommand& msg
    ) {
        return uavcan::equipment::esc::RPMCommand::decode(msg, rx_codec_);
    }

    bool decode_executeopcode_request(
        uavcan::protocol::param::ExecuteOpcode::Request& msg
    ) {
        return uavcan::protocol::param::ExecuteOpcode::Request::decode(
            msg, rx_codec_);
    }

    bool decode_getset_request(
        uavcan::protocol::param::GetSet::Request& msg
    ) {
        return uavcan::protocol::param::GetSet::Request::decode(
            msg, rx_codec_);
    }

    bool decode_restartnode_request(
        uavcan::protocol::RestartNode::Request& msg
    ) {
        return uavcan::protocol::RestartNode::Request::decode(msg, rx_codec_);
    }
};
