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

import os
import sys
import time
import serial
import struct
import binascii
import functools
import logging as log


UAVCAN_NODESTATUS_ID = 550


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

    return struct.unpack("<f", struct.pack("<L", t1))


def uavcan_broadcast_id(transfer_id, data_type_id):
    return (
        (transfer_id & 0x7) |
        (1 << 3) |  # Last frame = true
        (0 << 4) |  # Frame index = 0
        (127 << 10) |  # Source node ID = 0x7F
        (2 << 17) |  # Transfer type = broadcast
        ((data_type_id & 0x3FF) << 19)  # Data type ID
    )


def uavcan_get_node_id(message_id):
    # Extract the 7-bit node ID field from the message ID
    return (message_id >> 10) & 0x7F


def uavcan_get_data_type_id(message_id):
    return (message_id >> 19) & 0x3FF


class CAN(object):
    def __init__(self, device, baudrate=1000000):
        self.conn = serial.Serial(device, baudrate, timeout=0)
        self._read_handler = self._get_bytes_sync
        self.partial_message  =""

    def _get_bytes_sync(self):
        return self.conn.read(1)

    def _get_bytes_async(self):
        return os.read(self.conn.fd, 1024)

    def _ioloop_event_handler(self, fd, events, callback=None):
        self.recv(callback=callback)

    def add_to_ioloop(self, ioloop, callback=None):
        self._read_handler = self._get_bytes_async
        ioloop.add_handler(
            self.conn.fd,
            functools.partial(self._ioloop_event_handler, callback=callback),
            ioloop.READ)

    def parse(self, message):
        try:
            if message[0] == "T":
                id_len = 8
            else:
                id_len = 3

            # Parse the message into a (message ID, data) tuple.
            packet_id = int(message[1:1 + id_len], 16)
            packet_len = int(message[1 + id_len])
            packet_data = binascii.a2b_hex(message[2 + id_len:
                                                   2 + id_len + packet_len * 2])

            # ID, data, extended
            return packet_id, packet_data, (id_len == 8)
        except Exception:
            return None

    def open(self, callback=None):
        self.close()
        self.conn.write("S8\r")
        self.conn.flush()
        self.recv()
        time.sleep(0.1)
        self.conn.write("O\r")
        self.conn.flush()
        self.recv()
        time.sleep(0.1)

    def close(self, callback=None):
        self.conn.write("C\r")
        self.conn.flush()
        time.sleep(0.1)

    def recv(self, callback=None):
        bytes = ""
        new_bytes = self._read_handler()
        while new_bytes:
            bytes += new_bytes
            new_bytes = self._read_handler()

        if not bytes:
            if callback:
                return
            else:
                return []

        # Split into messages
        messages = [self.partial_message]
        for byte in bytes:
            if byte in "tT":
                messages.append(byte)
            elif messages and byte in "0123456789ABCDEF":
                messages[-1] += byte
            elif byte in "\x07\r":
                messages.append("")

        if messages[-1]:
            self.partial_message = messages.pop()
        # Filter, parse and return the messages
        messages = list(self.parse(m) for m in messages
                        if m and m[0] in ("t", "T"))
        messages = filter(lambda x: x and x[0], messages)

        if callback:
            for message in messages:
                #log.debug("CAN.recv(): {!r}".format(message))
                try:
                    callback(self, message)
                except Exception:
                    pass
        else:
            #for message in messages:
            #    log.debug("CAN.recv(): {!r}".format(message))
            return messages

    def send(self, message_id, message, extended=False):
        #log.debug("CAN.send({!r}, {!r}, {!r})".format(message_id, message,
        #                                              extended))
        if extended:
            start = "T{0:8X}".format(message_id)
        else:
            start = "t{0:3X}".format(message_id)
        line = "{0:s}{1:1d}{2:s}\r".format(start, len(message),
                                           binascii.b2a_hex(message))
        self.conn.write(line)
        self.conn.flush()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: can.py CAN_DEVICE")
        sys.exit()

    can = CAN(sys.argv[1])
    can.open()
    last_status = time.time()
    transfer_id = 0
    while True:
        messages = can.recv()
        for message in messages:
            if message[0]:
                print(repr(message))

        # Send node status every 0.5 s
        if time.time() - last_status > 0.5:
            message_id = uavcan_broadcast_id(transfer_id,
                                             UAVCAN_NODESTATUS_ID)
            can.send(message_id, "\x00\x00\x00\x00", extended=True)
            last_status = time.time()
            transfer_id += 1

