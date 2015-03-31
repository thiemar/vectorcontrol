#encoding=utf-8

# Copyright (c) 2014 - 2015 by Thiemar Pty Ltd
#
# This file is part of vectorcontrol.
#
# vectorcontrol is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# vectorcontrol. If not, see <http://www.gnu.org/licenses/>.


import can
import time
import math
import ctypes
import struct
import logging
import binascii
import functools
import collections


try:
    import tornado
    import tornado.gen
    import tornado.ioloop
    import tornado.concurrent
except ImportError:
    pass


def bits_from_bytes(s):
    return "".join(format(c, "08b") for c in s)


def bytes_from_bits(s):
    return bytearray(int(s[i:i+8], 2) for i in xrange(0, len(s), 8))


def swap_bytes(s):
    if len(s) > 8:
        return bits_from_bytes(bytes_from_bits(s)[::-1])
    else:
        return s


def format_bits(s):
    return " ".join(s[i:i+8] for i in xrange(0, len(s), 8))


# http://davidejones.com/blog/1413-python-precision-floating-point/
def f16_from_f32(float32):
    F16_EXPONENT_BITS = 0x1F
    F16_EXPONENT_SHIFT = 10
    F16_EXPONENT_BIAS = 15
    F16_MANTISSA_BITS = 0x3ff
    F16_MANTISSA_SHIFT =  (23 - F16_EXPONENT_SHIFT)
    F16_MAX_EXPONENT =  (F16_EXPONENT_BITS << F16_EXPONENT_SHIFT)

    a = struct.pack('>f', float32)
    b = binascii.hexlify(a)

    f32 = int(b, 16)
    f16 = 0
    sign = (f32 >> 16) & 0x8000
    exponent = ((f32 >> 23) & 0xff) - 127
    mantissa = f32 & 0x007fffff

    if exponent == 128:
        f16 = sign | F16_MAX_EXPONENT
        if mantissa:
            f16 |= (mantissa & F16_MANTISSA_BITS)
    elif exponent > 15:
        f16 = sign | F16_MAX_EXPONENT
    elif exponent > -15:
        exponent += F16_EXPONENT_BIAS
        mantissa >>= F16_MANTISSA_SHIFT
        f16 = sign | exponent << F16_EXPONENT_SHIFT | mantissa
    else:
        f16 = sign
    return f16


# http://davidejones.com/blog/1413-python-precision-floating-point/
def f32_from_f16(float16):
    t1 = float16 & 0x7FFF
    t2 = float16 & 0x8000
    t3 = float16 & 0x7C00

    t1 <<= 13
    t2 <<= 16

    t1 += 0x38000000
    t1 = 0 if t3 == 0 else t1
    t1 |= t2

    return struct.unpack("<f", struct.pack("<L", t1))[0]


class TransferType(object):
    SERVICE_RESPONSE = 0
    SERVICE_REQUEST = 1
    MESSAGE_BROADCAST = 2
    MESSAGE_UNICAST = 3


class NodeStatus(object):
    OK = 0
    INITIALIZING = 1
    WARNING = 2
    CRITICAL = 3
    OFFLINE = 15


class DataType(object):
    def __init__(self, bit_width=None):
        self._bit_width = bit_width
        self._bit_value = None

    def unpack(self, stream):
        if self._bit_width:
            self._bit_value = swap_bytes(stream[0:self._bit_width])
            return stream[self._bit_width:]
        else:
            return stream

    def pack(self):
        if self._bit_value and len(self._bit_value) >= self._bit_width:
            return swap_bytes(self._bit_value[0:self._bit_width])
        elif self._bit_value:
            return swap_bytes("0" * (self._bit_width - len(self._bit_value)) +
                              self._bit_value)
        else:
            return "0" * self._bit_width


class IntegerType(DataType):
    def __init__(self, saturate=False, *args, **kwargs):
        super(IntegerType, self).__init__(*args, **kwargs)
        self._saturate = saturate

    def __repr__(self):
        return "IntegerType(bit_width={0!r}, value={1!r})".format(
                self._bit_width, self.value)

    def __str__(self):
        return self.__repr__()

    @property
    def value(self):
        if not self._bit_value:
            return None
        return int(self._bit_value, 2)

    @value.setter
    def value(self, new_value):
        # TODO: support saturation
        self._bit_value = bin(new_value)[2:]


class FloatType(DataType):
    def __repr__(self):
        return "FloatType(bit_width={0!r}, value={1!r})".format(
                self._bit_width, self.value)

    def __str__(self):
        return self.__repr__()

    @property
    def value(self):
        if not self._bit_value:
            return None
        int_value = int(self._bit_value, 2)
        if self._bit_width == 16:
            return f32_from_f16(int_value)
        elif self._bit_width == 32:
            return struct.unpack("<f", struct.pack("<L", int_value))[0]
        else:
            raise RuntimeError("Float width must be either 16 or 32 bits")

    @value.setter
    def value(self, new_value):
        if self._bit_width == 16:
            self._bit_value = bin(f16_from_f32(new_value))[2:]
        elif self._bit_width == 32:
            int_value = struct.unpack("<L", struct.pack("<f", new_value))[0]
            self._bit_value = bin(int_value)[2:]
        else:
            raise RuntimeError("Float width must be either 16 or 32 bits")


class BoolType(DataType):
    def __init__(self, *args, **kwargs):
        super(BoolType, self).__init__(bit_width=1, *args, **kwargs)

    def __repr__(self):
        return "BoolType(value={0!r})".format(self.value)

    def __str__(self):
        return self.__repr__()

    @property
    def value(self):
        if not self._bit_value:
            return None
        return self._bit_value == '1'

    @value.setter
    def value(self, new_value):
        self._bit_value = '1' if new_value else '0'


class ArrayType(collections.MutableSequence):
    def __init__(self, item_ctor, max_items=255, tao=False, *args, **kwargs):
        super(ArrayType, self).__init__(*args, **kwargs)
        self._max_items = max_items
        self._tao = tao
        self.__item_ctor = item_ctor
        self.__items = []

    def __repr__(self):
        return "ArrayType(max_items={0!r}, tao={1!r}, items={2!r})".format(
                self._max_items, self._tao, self.__items)

    def __str__(self):
        return self.__repr__()

    def __getitem__(self, idx):
        return self.__items[idx]

    def __setitem__(self, idx, value):
        if idx >= self._max_items:
            raise IndexError(("Index {0} too large (max items " +
                              "{1})").format(idx, self._max_items))
        self.__items[idx] = value

    def __delitem__(self, idx):
        del self.__items[idx]

    def __len__(self):
        return len(self.__items)

    def insert(self, idx, value):
        if idx >= self._max_items:
            raise IndexError(("Index {0} too large (max items " +
                              "{1})").format(idx, self._max_items))
        elif len(self) == self._max_items:
            raise IndexError(("Array already full (max items "
                              "{0})").format(self._max_items))
        self.__items.insert(idx, value)

    def unpack(self, stream):
        if self._tao:
            while len(stream) >= 8:
                new_item = self.__item_ctor()
                stream = new_item.unpack(stream)
                self.append(new_item)

            return ""
        else:
            count_width = int(math.ceil(math.log(self._max_items, 2))) or 1
            count = IntegerType(bit_width=count_width)
            stream = count.unpack(stream)

            for i in xrange(count.value):
                new_item = self.__item_ctor()
                stream = new_item.unpack(stream)
                self.append(new_item)

            return stream

    def pack(self):
        if self._tao:
            return "".join(i.pack() for i in self)
        else:
            count_width = int(math.ceil(math.log(self._max_items, 2))) or 1
            count = format(len(self), "0{0:1d}b".format(count_width))
            return count + "".join(i.pack() for i in self)


class StringType(bytearray):
    def __init__(self, max_length=255, tao=False, *args, **kwargs):
        super(StringType, self).__init__(*args, **kwargs)
        self._max_length = max_length
        self._tao = tao

    def __repr__(self):
        return "StringType(max_length={0!r}, tao={1!r}, value='{2!s}')".format(
                self._max_length, self._tao, self)

    def unpack(self, stream):
        if self._tao:
            count = int(len(stream) / 8) * 8
        else:
            count_width = int(math.ceil(math.log(self._max_length, 2))) or 1
            count = IntegerType(bit_width=count_width)
            stream = count.unpack(stream)
            count = count.value

        self.extend(bytes_from_bits(stream[0:count]))
        return stream[count:]

    def pack(self):
        if self._tao:
            return bits_from_bytes(self)
        else:
            count_width = int(math.ceil(math.log(self._max_length, 2))) or 1
            count = format(len(self), "0{0:1d}b".format(count_width))
            return count + bits_from_bytes(self)


class AggregateType(DataType):
    def __init__(self, **kwargs):
        self.__dict__["fields"] = tuple()
        super(AggregateType, self).__init__(**kwargs)

    def __getattr__(self, attr):
        field_map = dict(self.fields)
        if attr in field_map:
            if isinstance(field_map[attr], DataType) and \
                    not isinstance(field_map[attr], AggregateType):
                return field_map[attr].value
            elif isinstance(field_map[attr], StringType):
                return field_map[attr].decode("utf-8")
            else:
                return field_map[attr]
        else:
            raise AttributeError(attr)

    def __setattr__(self, attr, value):
        field_map = dict(self.fields)
        if attr in field_map:
            if isinstance(field_map[attr], StringType):
                field_map[attr][:] = value.encode("utf-8")
            else:
                field_map[attr].value = value
        else:
            super(AggregateType, self).__setattr__(attr, value)

    def unpack(self, stream):
        for field in self.fields:
            stream = field[1].unpack(stream)
        return stream

    def pack(self):
        return "".join(field[1].pack() for field in self.fields)


class ParamValueType(AggregateType):
    def __init__(self, **kwargs):
        super(ParamValueType, self).__init__(**kwargs)
        self.fields = (
            ("value_bool", ArrayType(BoolType, max_items=1)),
            ("value_int",  ArrayType(
                                functools.partial(IntegerType, bit_width=64),
                                max_items=1)),
            ("value_float", ArrayType(
                                functools.partial(FloatType, bit_width=32),
                                max_items=1))
        )

    @property
    def bool_value(self):
        if self.fields[0][1]:
            return self.fields[0][1][0].value
        else:
            return None

    @bool_value.setter
    def bool_value(self, value):
        del self.fields[0][1][:]
        if value:
            val = BoolType()
            val.value = value
            self.fields[0][1].append(val)

    @property
    def int_value(self):
        if self.fields[1][1]:
            return self.fields[1][1][0].value
        else:
            return None

    @int_value.setter
    def int_value(self, value):
        del self.fields[1][1][:]
        if value:
            val = IntegerType(bit_width=64)
            val.value = value
            self.fields[1][1].append(val)

    @property
    def float_value(self):
        if self.fields[2][1]:
            return self.fields[2][1][0].value
        else:
            return None

    @float_value.setter
    def float_value(self, value):
        del self.fields[2][1][:]
        if value:
            val = FloatType(bit_width=32)
            val.value = value
            self.fields[2][1].append(val)


class ParamGetSetRequest(AggregateType):
    DATA_TYPE_ID = 599
    DATA_TYPE_SIGNATURE = 0xA9314B70D8AA0726
    TRANSFER_TYPE = TransferType.SERVICE_REQUEST

    def __init__(self, *args, **kwargs):
        super(ParamGetSetRequest, self).__init__(*args, **kwargs)
        self.fields = (
            ("value", ParamValueType()),
            ("index", IntegerType(bit_width=8)),
            ("name", StringType(max_length=40, tao=True))
        )


class ParamGetSetResponse(AggregateType):
    DATA_TYPE_ID = 599
    DATA_TYPE_SIGNATURE = 0xA9314B70D8AA0726
    TRANSFER_TYPE = TransferType.SERVICE_RESPONSE

    def __init__(self, *args, **kwargs):
        super(ParamGetSetResponse, self).__init__(*args, **kwargs)
        self.fields = (
            ("value", ParamValueType()),
            ("default", ParamValueType()),
            ("max", ParamValueType()),
            ("min", ParamValueType()),
            ("name", StringType(max_length=40, tao=True))
        )


class ParamSaveEraseRequest(AggregateType):
    DATA_TYPE_ID = 598
    DATA_TYPE_SIGNATURE = 0
    TRANSFER_TYPE = TransferType.SERVICE_REQUEST

    def __init__(self, *args, **kwargs):
        super(ParamSaveEraseRequest, self).__init__(*args, **kwargs)
        self.fields = (
            ("opcode", IntegerType(bit_width=2)),
        )

    @property
    def save(self):
        return self.opcode == 0

    @save.setter
    def save(self, value):
        if value:
            self.opcode = 0
        else:
            self.opcode = 1

    @property
    def erase(self):
        return self.opcode == 1

    @erase.setter
    def erase(self, value):
        if value:
            self.opcode = 1
        else:
            self.opcode = 0


class ParamSaveEraseResponse(AggregateType):
    DATA_TYPE_ID = 598
    DATA_TYPE_SIGNATURE = 0
    TRANSFER_TYPE = TransferType.SERVICE_RESPONSE

    def __init__(self, *args, **kwargs):
        super(ParamSaveEraseResponse, self).__init__(*args, **kwargs)
        self.fields = (
            ("ok", BoolType()),
        )


class RestartNodeRequest(AggregateType):
    DATA_TYPE_ID = 560
    DATA_TYPE_SIGNATURE = 0
    TRANSFER_TYPE = TransferType.SERVICE_REQUEST

    def __init__(self, *args, **kwargs):
        super(RestartNodeRequest, self).__init__(*args, **kwargs)
        self.fields = (
            ("magic_number", IntegerType(bit_width=40)),
        )
        self.fields[0][1].value = 0xACCE551B1E


class RestartNodeResponse(AggregateType):
    DATA_TYPE_ID = 560
    DATA_TYPE_SIGNATURE = 0
    TRANSFER_TYPE = TransferType.SERVICE_RESPONSE

    def __init__(self, *args, **kwargs):
        super(RestartNodeResponse, self).__init__(*args, **kwargs)
        self.fields = (
            ("ok", BoolType()),
        )


class GetNodeInfoRequest(AggregateType):
    DATA_TYPE_ID = 551
    DATA_TYPE_SIGNATURE = 0xE73283A5632ECF99
    TRANSFER_TYPE = TransferType.SERVICE_REQUEST

    def __init__(self, *args, **kwargs):
        super(GetNodeInfoRequest, self).__init__(*args, **kwargs)


class GetNodeInfoResponse(AggregateType):
    DATA_TYPE_ID = 551
    DATA_TYPE_SIGNATURE = 0xE73283A5632ECF99
    TRANSFER_TYPE = TransferType.SERVICE_RESPONSE

    def __init__(self, *args, **kwargs):
        super(GetNodeInfoRequest, self).__init__(*args, **kwargs)


class NodeStatusBroadcast(AggregateType):
    DATA_TYPE_ID = 550
    DATA_TYPE_SIGNATURE = 0
    TRANSFER_TYPE = TransferType.MESSAGE_BROADCAST

    def __init__(self, *args, **kwargs):
        super(NodeStatusBroadcast, self).__init__(*args, **kwargs)
        self.fields = (
            ("uptime_sec", IntegerType(bit_width=28)),
            ("status_code", IntegerType(bit_width=4))
        )


class Frame(object):
    def __init__(self, message_id=None, raw_payload=None):
        self.message_id = message_id
        self._payload = bytearray(raw_payload) if raw_payload else None

    def __str__(self):
        return ("Frame(message_id={0!r}, raw_payload={1!r}): " +
                "transfer_id={2}, last_frame={3}, frame_index={4}, " +
                "source_node_id={5}, transfer_type={6}, data_type_id={7}, " +
                "dest_node_id={8}, payload={9}").format(
                self.message_id, self._payload, self.transfer_id,
                self.last_frame, self.frame_index, self.source_node_id,
                self.transfer_type, self.data_type_id, self.dest_node_id,
                format_bits(bits_from_bytes(self.payload)))

    def __repr__(self):
        return "Frame(message_id={0!r}, raw_payload={1!r})".format(
                self.message_id, self._payload)

    @property
    def transfer_id(self):
        return self.message_id & 0x7

    @transfer_id.setter
    def transfer_id(self, value):
        self.message_id = (self.message_id & ~0x7) | (value & 0x7)

    @property
    def last_frame(self):
        return True if ((self.message_id >> 3) & 0x1) else False

    @last_frame.setter
    def last_frame(self, value):
        self.message_id = (self.message_id & ~(0x1 << 3)) | \
                          ((1 if value else 0) << 3)

    @property
    def frame_index(self):
        return (self.message_id >> 4) & 0x3F

    @frame_index.setter
    def frame_index(self, value):
        self.message_id = (self.message_id & ~(0x3F << 4)) | \
                          ((value & 0x3F) << 4)

    @property
    def source_node_id(self):
        return (self.message_id >> 10) & 0x7F

    @source_node_id.setter
    def source_node_id(self, value):
        self.message_id = (self.message_id & ~(0x7F << 10)) | \
                          ((value & 0x7F) << 10)

    @property
    def transfer_type(self):
        return (self.message_id >> 17) & 0x3

    @transfer_type.setter
    def transfer_type(self, value):
        self.message_id = (self.message_id & ~(0x3 << 17)) | \
                          ((value & 0x3) << 17)

    @property
    def data_type_id(self):
        return (self.message_id >> 19) & 0x3FF

    @data_type_id.setter
    def data_type_id(self, value):
        self.message_id = (self.message_id & ~(0x3FF << 19)) | \
                          ((value & 0x3FF) << 19)

    @property
    def dest_node_id(self):
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            return 0
        else:
            return self._payload[0] & 0x7F

    @dest_node_id.setter
    def dest_node_id(self, value):
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            raise ValueError("Can't set dest_node_id for a broadcast frame")
        elif not self.payload:
            self._payload = bytearray([value & 0x7F])
        else:
            self._payload[0] = value & 0x7F

    @property
    def payload(self):
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            return self._payload
        elif self._payload:
            return self._payload[1:]
        else:
            return None

    @payload.setter
    def payload(self, value):
        value = bytearray(value)
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            if len(value) > 8:
                raise IndexError("Maximum broadcast frame payload length is 8")
            else:
                self._payload = value
        else:
            if len(value) > 7:
                raise IndexError("Maximum non-broadcast frame payload length is 7")
            elif self._payload:
                self._payload[1:] = value
            else:
                self._payload = bytearray([0]) + value

    @property
    def transfer_key(self):
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            return (self.source_node_id, self.data_type_id, self.transfer_id)
        else:
            return (self.source_node_id, self.dest_node_id, self.data_type_id,
                    self.transfer_id)

    def to_bytes(self):
        return self._payload


class Transfer(object):
    def __init__(self, transfer_id=0, transfer_type=0,
                 source_node_id=0, data_type_id=0, dest_node_id=0,
                 payload=0):
        self.transfer_id = transfer_id
        self.transfer_type = transfer_type
        self.source_node_id = source_node_id
        self.data_type_id = data_type_id
        self.dest_node_id = dest_node_id
        self.data_type_signature = 0

        if isinstance(payload, AggregateType):
            payload_bits = payload.pack()
            if len(payload_bits) & 7:
                payload_bits += "0" * (8 - (len(payload_bits) & 7))
            self.payload = bytes_from_bits(payload_bits)
            self.data_type_id = payload.DATA_TYPE_ID
            self.data_type_signature = payload.DATA_TYPE_SIGNATURE
            self.transfer_type = payload.TRANSFER_TYPE
        else:
            self.payload = payload

        self.is_complete = True if self.payload else False

    def calculate_crc(self):
        crc = 0xFFFF
        for byte in (struct.pack("<Q", self.data_type_signature) + self.payload):
            crc ^= byte << 8
            for bit in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc & 0xFFFF

    def to_frames(self):
        # Broadcast frames support up to 8 bytes, other frames have a
        # destination node ID which consumes the first byte of the message
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            bytes_per_frame = 8
        else:
            bytes_per_frame = 7

        out_frames = []
        remaining_payload = self.payload

        # Prepend the transfer CRC to the payload if the transfer requires
        # multiple frames
        if len(remaining_payload) > bytes_per_frame:
            transfer_crc = self.calculate_crc()
            remaining_payload = bytearray([transfer_crc & 0xFF,
                                           transfer_crc >> 8]) + \
                                remaining_payload

        # Generate the frame sequence
        while True:
            frame = Frame(message_id=0)
            frame.transfer_id = self.transfer_id
            frame.frame_index = len(out_frames)
            frame.last_frame = len(remaining_payload) <= bytes_per_frame
            frame.source_node_id = self.source_node_id
            frame.data_type_id = self.data_type_id
            if self.transfer_type != TransferType.MESSAGE_BROADCAST:
                frame.dest_node_id = self.dest_node_id
            frame.payload = remaining_payload[0:bytes_per_frame]

            out_frames.append(frame)
            remaining_payload = remaining_payload[bytes_per_frame:]
            if not remaining_payload:
                break

        return out_frames

    def from_frames(self, frames):
        for i, f in enumerate(frames):
            if i != f.frame_index:
                raise IndexError(("Frame index mismatch: expected {0}, " +
                                  "got {1}").format(i, f.frame_index))

        self.transfer_id = frames[0].transfer_id
        self.transfer_type = frames[0].transfer_type
        self.source_node_id = frames[0].source_node_id
        self.data_type_id = frames[0].data_type_id
        if self.transfer_type != TransferType.MESSAGE_BROADCAST:
            self.dest_node_id = frames[0].dest_node_id
        self.payload = sum((f.payload for f in frames), bytearray())

        # For a multi-frame transfer, validate the CRC and frame indexes
        if len(frames) > 1:
            for cls in AggregateType.__subclasses__():
                if getattr(cls, "DATA_TYPE_ID", 0) == self.data_type_id:
                    self.data_type_signature = \
                        getattr(cls, "DATA_TYPE_SIGNATURE", 0)

            transfer_crc = self.payload[0] + (self.payload[1] << 8)
            self.payload = self.payload[2:]
            expected_crc = self.calculate_crc()
            if transfer_crc != expected_crc:
                raise ValueError("CRC mismatch: expected {0:x}, got {1:x}".format(
                                 expected_crc, transfer_crc))

    @property
    def key(self):
        if self.transfer_type == TransferType.MESSAGE_BROADCAST:
            return (self.source_node_id, self.data_type_id, self.transfer_id)
        else:
            return (self.source_node_id, self.dest_node_id, self.data_type_id,
                    self.transfer_id)

    def is_response_to(self, transfer):
        if self.transfer_type == TransferType.SERVICE_RESPONSE and \
                self.source_node_id == transfer.dest_node_id and \
                self.dest_node_id == transfer.source_node_id and \
                self.data_type_id == transfer.data_type_id and \
                self.transfer_id == transfer.transfer_id:
            return True
        else:
            return False


class TransferManager(object):
    def __init__(self):
        self.active_transfers = collections.defaultdict(list)
        self.active_transfer_timestamps = {}

    def receive_frame(self, frame):
        key = frame.transfer_key
        self.active_transfers[key].append(frame)
        self.active_transfer_timestamps[key] = time.time()

        # If the last frame of a transfer was received, process it.
        if frame.last_frame:
            try:
                transfer = Transfer()
                transfer.from_frames(self.active_transfers[key])
                return transfer
            except:
                pass  # FIXME
            finally:
                del self.active_transfers[key]
                del self.active_transfer_timestamps[key]
        else:
            return None

    def remove_inactive_transfers(self, timeout=1.0):
        t = time.time()
        transfer_keys = self.active_transfers.keys()
        for key in transfer_keys:
            if t - self.active_transfer_timestamps[key] > timeout:
                del self.active_transfers[key]
                del self.active_transfer_timestamps[key]


class Application(object):
    def __init__(self, handlers, node_id=127):
        self.can = None
        self.transfer_manager = TransferManager()
        self.handlers = handlers
        self.node_id = node_id
        self.outstanding_requests = {}
        self.outstanding_request_callbacks = {}
        self.outstanding_request_timestamps = {}
        self.next_transfer_ids = collections.defaultdict(int)
        self.node_info = {}

    def _recv_frame(self, dev, message):
        frame_id, frame_data, ext_id = message
        if not ext_id:
            return

        frame = Frame(frame_id, frame_data)
        logging.debug("Application._recv_frame(): got {0!s}".format(frame))

        transfer = self.transfer_manager.receive_frame(frame)
        if not transfer:
            return

        # Try to find the appropriate data type payload class, based on the
        # data type ID and transfer type
        payload = None
        for cls in AggregateType.__subclasses__():
            if getattr(cls, "DATA_TYPE_ID", 0) == transfer.data_type_id and \
                    getattr(cls, "TRANSFER_TYPE", 0) == transfer.transfer_type:
                payload = cls()
                payload.unpack(bits_from_bytes(transfer.payload))

        # If it's a node info request, keep track of the status of each node
        if transfer.data_type_id == NodeStatusBroadcast.DATA_TYPE_ID:
            self.node_info[transfer.source_node_id] = {
                "uptime": payload.uptime_sec,
                "status": payload.status_code,
                "timestamp": time.time()
            }
            logging.debug(
                "Application._recv_frame(): got node info {0!r}".format(
                self.node_info[transfer.source_node_id]))

        if transfer.transfer_type != TransferType.SERVICE_RESPONSE and \
                (transfer.transfer_type == TransferType.MESSAGE_BROADCAST or
                 transfer.dest_node_id == self.node_id):
            # This is a request, a unicast or a broadcast; look up the
            # appropriate handler by data type ID
            for handler in self.handlers:
                if handler[0] == transfer.data_type_id:
                    handler[1](payload, transfer)
        elif transfer.transfer_type == TransferType.SERVICE_RESPONSE and \
                transfer.dest_node_id == self.node_id:
            # This is a reply to a request we sent. Look up the original
            # request and call the appropriate callback
            requests = self.outstanding_requests.keys()
            for key in requests:
                if transfer.is_response_to(self.outstanding_requests[key]):
                    # Call the request's callback and remove it from the
                    # active list
                    if key in self.outstanding_request_callbacks:
                        self.outstanding_request_callbacks[key]((payload, transfer))
                        del self.outstanding_request_callbacks[key]
                    del self.outstanding_requests[key]
                    del self.outstanding_request_timestamps[key]
                    break

    def listen(self, device):
        self.can = can.CAN(device)
        self.can.add_to_ioloop(tornado.ioloop.IOLoop.current(),
                               callback=self._recv_frame)

    @tornado.concurrent.return_future
    def send_request(self, payload, dest_node_id=None, callback=None):
        transfer_id = self.next_transfer_ids[(payload.DATA_TYPE_ID, dest_node_id)]
        transfer = Transfer(
            payload=payload,
            source_node_id=self.node_id,
            dest_node_id=dest_node_id,
            transfer_id=transfer_id)
        self.next_transfer_ids[(payload.DATA_TYPE_ID, dest_node_id)] = \
            (transfer_id + 1) & 7
        for frame in transfer.to_frames():
            self.can.send(frame.message_id, frame.to_bytes(), extended=True)

        self.outstanding_requests[transfer.key] = transfer
        self.outstanding_request_callbacks[transfer.key] = callback
        self.outstanding_request_timestamps[transfer.key] = time.time()

    def send_unicast(self, payload, dest_node_id=None):
        transfer_id = self.next_transfer_ids[(payload.DATA_TYPE_ID, dest_node_id)]
        transfer = Transfer(
            payload=payload,
            source_node_id=self.node_id,
            dest_node_id=dest_node_id,
            transfer_id=transfer_id)
        self.next_transfer_ids[(payload.DATA_TYPE_ID, dest_node_id)] = \
            (transfer_id + 1) & 7
        for frame in transfer.to_frames():
            self.can.send(frame.message_id, frame.to_bytes(), extended=True)

    def send_broadcast(self, payload):
        transfer_id = self.next_transfer_ids[payload.DATA_TYPE_ID]
        transfer = Transfer(
            payload=payload,
            source_node_id=self.node_id,
            transfer_id=transfer_id)
        self.next_transfer_ids[payload.DATA_TYPE_ID] = (transfer_id + 1) & 7
        for frame in transfer.to_frames():
            self.can.send(frame.message_id, frame.to_bytes(), extended=True)

