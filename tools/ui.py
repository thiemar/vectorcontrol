#encoding=utf-8

# Copyright (c) 2014 - 2015 Thiemar Pty Ltd
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
import cgi
import sys
import code
import json
import stat
import time
import math
import struct
import binascii
import datetime
import cStringIO
import functools
import tornado.web
import collections
import logging as log
import tornado.ioloop
import tornado.netutil
import tornado.template
from tornado import gen
import tornado.websocket
import tornado.httpserver
from optparse import OptionParser, OptionGroup


sys.path.insert(0, "/Users/bendyer/Projects/ARM/workspace/pyuavcan/")


import uavcan
import uavcan.node
import uavcan.handlers


NOTIFY_SOCKETS = set()
UAVCAN_NODE_UPTIME = {}
UAVCAN_NODE_CONFIG = collections.defaultdict(dict)
UAVCAN_NODE_SETPOINT_TIMER = collections.defaultdict(int)
UAVCAN_NODE_SETPOINT_SCHEDULE = collections.defaultdict(list)
UAVCAN_NODE_SETPOINT_STARTUP = collections.defaultdict(list)
UAVCAN_NODE_CONTROL_MODE = collections.defaultdict(int)
UAVCAN_NODE_MOTOR_RUNNING = collections.defaultdict(bool)
UAVCAN_NODE_CONTROLLER_QUEUE = collections.defaultdict(collections.deque)
UAVCAN_NODE_MEASUREMENT_QUEUE = collections.defaultdict(collections.deque)
UAVCAN_NODE_VOLTAGE_QUEUE = collections.defaultdict(collections.deque)
UAVCAN_NODE_HFI_QUEUE = collections.defaultdict(collections.deque)


SETPOINT_SCHEDULE_TIMER = 0


def send_all(message):
    global NOTIFY_SOCKETS
    for socket in NOTIFY_SOCKETS:
        socket.write_message(message)


def handle_can_message(conn, message):
    """Process an incoming CAN message."""
    global UAVCAN_NODES, UAVCAN_NODE_MEASUREMENT_QUEUE, \
           UAVCAN_NODE_CONTROLLER_QUEUE, UAVCAN_NODE_UPTIME, \
           UAVCAN_NODE_HFI_QUEUE

    message_id = message[0]
    message_data = message[1]
    extended = message[2]

    if extended:
        data_type_id = can.uavcan_get_data_type_id(message_id)
    else:
        data_type_id = None

    if data_type_id == UAVCAN_NODESTATUS_ID:
        node_id = can.uavcan_get_node_id(message_id)
        u0, u1, u2, status = struct.unpack("<4B", message_data)
        uptime = u0 + (u1 << 8) + (u2 << 16)

        if node_id not in UAVCAN_NODE_UPTIME or \
                UAVCAN_NODE_UPTIME[node_id] > uptime:
            # New node -- get configuration in the next IOLoop iteration
            log.debug(("handle_can_message(): detected new/restarted " +
                       "node ID {0}").format(node_id))
            tornado.ioloop.IOLoop.instance().add_callback(
                read_node_configuration, conn, node_id)

        UAVCAN_NODE_UPTIME[node_id] = uptime

        send_all({
            "node_id": node_id,
            "uptime": uptime
        })
    elif message_id == flash.CAN_STATUS_CONFIG_ID:
        node_id, param_index, value = struct.unpack("<BBf", message_data)
        param_name = param_name_from_param_index(param_index)
        UAVCAN_NODE_CONFIG[node_id][param_name] = value

        send_all({
            "node_id": node_id,
            "param_name": param_name,
            "value": value
        })
    elif message_id in (CAN_STATUS_CONTROLLER_ID, CAN_STATUS_MEASUREMENT_ID,
                        CAN_STATUS_VOLTAGE_ID, CAN_STATUS_HFI_ID):
        queue = key = node_id = None
        data = {}

        if message_id == CAN_STATUS_CONTROLLER_ID:
            node_id, i_d, i_q, i_setpoint = struct.unpack("<BHHH", message_data)
            data["i_d"] = can.f32_from_f16(i_d)
            data["i_q"] = can.f32_from_f16(i_q)
            data["i_setpoint"] = can.f32_from_f16(i_setpoint)
            queue = UAVCAN_NODE_CONTROLLER_QUEUE
            key = "current"
        elif message_id == CAN_STATUS_MEASUREMENT_ID:
            node_id, temperature, rpm_setpoint, rpm, vbus = \
                struct.unpack("<BbhhH", message_data)
            data["temperature"] = temperature
            data["rpm_setpoint"] = rpm_setpoint
            data["rpm"] = rpm
            data["vbus"] = can.f32_from_f16(vbus)
            queue = UAVCAN_NODE_MEASUREMENT_QUEUE
            key = "speed"
        elif message_id == CAN_STATUS_VOLTAGE_ID:
            node_id, v_d, v_q, consistency = struct.unpack("<BHHH", message_data)
            data["v_d"] = can.f32_from_f16(v_d)
            data["v_q"] = can.f32_from_f16(v_q)
            data["consistency"] = can.f32_from_f16(consistency)
            queue = UAVCAN_NODE_VOLTAGE_QUEUE
            key = "voltage"
        elif message_id == CAN_STATUS_HFI_ID:
            node_id, hfi_d, hfi_q, angle = struct.unpack("<BHHH", message_data)
            data["hfi_d"] = can.f32_from_f16(hfi_d)
            data["hfi_q"] = can.f32_from_f16(hfi_q)
            data["angle"] = can.f32_from_f16(angle)
            queue = UAVCAN_NODE_HFI_QUEUE
            key = "hfi"

        queue[node_id].append(data)
        if len(queue[node_id]) == 10:
            send_all({
                "node_id": node_id,
                key: list(queue[node_id])
            })
            queue[node_id].clear()


def handle_timer(conn):
    """Send setpoint update messages for each connected node based on the
    timer interval."""
    global UAVCAN_NODE_SETPOINT_TIMER, UAVCAN_NODE_SETPOINT_SCHEDULE, \
           UAVCAN_NODE_CONTROL_MODE, UAVCAN_NODE_SETPOINT_STARTUP, \
           UAVCAN_NODE_MOTOR_RUNNING

    for node_id, schedule in UAVCAN_NODE_SETPOINT_SCHEDULE.iteritems():
        if not UAVCAN_NODE_MOTOR_RUNNING[node_id]:
            # Any value other than 2 or 3 will stop the motor
            setpoint = 0
            mode = 0
            UAVCAN_NODE_SETPOINT_TIMER[node_id] = 0
        elif len(UAVCAN_NODE_SETPOINT_STARTUP[node_id]) > \
                UAVCAN_NODE_SETPOINT_TIMER[node_id]:
            # Start up gracefully, consuming startup entries as we go
            setpoint = UAVCAN_NODE_SETPOINT_STARTUP[node_id][UAVCAN_NODE_SETPOINT_TIMER[node_id]]
            mode = UAVCAN_NODE_CONTROL_MODE.get(node_id) or 3
            UAVCAN_NODE_SETPOINT_TIMER[node_id] += 1
        else:
            # Normal operation, loop over the setpoint schedule
            setpoint = schedule[(UAVCAN_NODE_SETPOINT_TIMER[node_id] -
                                 len(UAVCAN_NODE_SETPOINT_STARTUP[node_id])) % len(schedule)]
            mode = UAVCAN_NODE_CONTROL_MODE.get(node_id) or 3
            UAVCAN_NODE_SETPOINT_TIMER[node_id] += 1

        # log.debug("handle_timer(): node {0} setpoint {1}".format(node_id, setpoint))

        #message = struct.pack("<BBhh", node_id, int(mode), int(setpoint),
        #                      int(setpoint))
        #conn.send(CAN_COMMAND_SETPOINT_ID, message)


class MessageRelayHandler(uavcan.node.MessageHandler):
    def on_message(self, message):
        send_all({
            "datatype": message.type.get_normalized_definition(),
            "node_id": self.transfer.source_node_id,
            "payload": message
        })


class CANHandler(tornado.websocket.WebSocketHandler):
    def __init__(self, *args, **kwargs):
        self.can = kwargs.pop("can", None)
        self.node = kwargs.pop("node", None)
        super(CANHandler, self).__init__(*args, **kwargs)

    def check_origin(self, origin):
        return True

    def open(self):
        self.set_nodelay(True)
        log.info("CANHandler.open()")

    def on_message(self, message):
        global UAVCAN_NODE_SETPOINT_SCHEDULE, UAVCAN_NODE_CONTROL_MODE, \
               NOTIFY_SOCKETS, UAVCAN_NODE_MOTOR_RUNNING, \
               UAVCAN_NODE_SETPOINT_STARTUP

        message = json.loads(message)

        log.info("CANHandler.on_message({0})".format(repr(message)))

        # If this is the first message from a socket, add it to the notify
        # list and send the current configuration.
        if self not in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.add(self)

            for node_id, uptime in UAVCAN_NODE_UPTIME.iteritems():
                self.write_message({
                    "node_id": node_id,
                    "uptime": uptime
                })

            for node_id, params in UAVCAN_NODE_CONFIG.iteritems():
                for param_name, param_value in params.iteritems():
                    self.write_message({
                        "node_id": node_id,
                        "param_name": param_name,
                        "value": param_value
                    })

        if message.get("node_id") is None:
            return

        if "schedule" in message and \
                not UAVCAN_NODE_MOTOR_RUNNING[message["node_id"]]:
            start_setpoint = message["schedule"][0]
            UAVCAN_NODE_SETPOINT_STARTUP[message["node_id"]] = \
                list(start_setpoint * i / 100.0 for i in xrange(100))
            UAVCAN_NODE_SETPOINT_SCHEDULE[message["node_id"]] = \
                message["schedule"]
        elif "param_name" in message:
            index = param_index_from_param_name(message["param_name"])
            command = struct.pack("<BBBBf", message["node_id"], index, 1, 0,
                                  message["param_value"])
            self.can.send(flash.CAN_COMMAND_CONFIG_ID, command)
        elif "param_apply" in message:
            command = struct.pack("<BBBBf", message["node_id"], 0, 0, 1, 0)
            self.can.send(flash.CAN_COMMAND_CONFIG_ID, command)
            self.can.send(flash.CAN_COMMAND_RESTART_ID,
                          chr(message["node_id"]) + "\xD4\x6A\xDD\x6E")
        elif "mode" in message:
            UAVCAN_NODE_CONTROL_MODE[message["node_id"]] = message["mode"]
        elif "motor_running" in message:
            UAVCAN_NODE_MOTOR_RUNNING[message["node_id"]] = \
                message["motor_running"]

    def on_close(self):
        global NOTIFY_SOCKETS

        log.info("CANHandler.on_close()")
        if self in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.remove(self)


class UI(tornado.web.RequestHandler):
    _cache_time = 1800  # 30 minutes

    def initialize(self, environment=None):
        self.environment = environment

    # Stupid override to stop Tornado removing whitespace from the template
    def create_template_loader(self, template_path):
        if "template_loader" in self.application.settings:
            return self.application.settings["template_loader"]

        opts = {}
        if "autoescape" in self.application.settings:
            opts["autoescape"] = self.application.settings["autoescape"]

        class Loader(tornado.template.Loader):
            def _create_template(self, name):
                with open(os.path.join(self.root, name), "rb") as f:
                    template = tornado.template.Template(f.read(), name=name,
                        loader=self, compress_whitespace=False)
                return template

        return Loader(template_path, **opts)

    def get(self):
        self.render("ui.html", environment=self.environment)


class AppDescriptor(object):
    """
    UAVCAN firmware image descriptor format:
    uint64_t signature (bytes [7:0] set to 'APDesc00' by linker script)
    uint64_t image_crc (set to 0 by linker script)
    uint32_t image_size (set to 0 by linker script)
    uint32_t vcs_commit (set to 0 by linker script)
    uint8_t version_major (set in source)
    uint8_t version_minor (set in source)
    uint8_t reserved[6] (set to 0xFF by linker script)
    """

    LENGTH = 8 + 8 + 4 + 4 + 1 + 1 + 6
    SIGNATURE = b"APDesc00"
    RESERVED =  b"\xFF" * 6

    def __init__(self, bytes=None):
        self.signature = AppDescriptor.SIGNATURE
        self.image_crc = 0
        self.image_size = 0
        self.vcs_commit = 0
        self.version_major = 0
        self.version_minor = 0
        self.reserved = AppDescriptor.RESERVED

        if bytes:
            try:
                self.unpack(bytes)
            except Exception:
                raise ValueError("Invalid AppDescriptor: {0}".format(
                                 binascii.b2a_hex(bytes)))

    def pack(self):
        return struct.pack("<8sQLLBB6s", self.signature, self.image_crc,
                           self.image_size, self.vcs_commit,
                           self.version_major, self.version_minor,
                           self.reserved)

    def unpack(self, bytes):
        (self.signature, self.image_crc, self.image_size, self.vcs_commit,
            self.version_major, self.version_minor, self.reserved) = \
            struct.unpack("<8sQLLBB6s", bytes)

        if not self.empty and not self.valid:
            raise ValueError()

    @property
    def empty(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc == 0 and self.image_size == 0 and
                self.vcs_commit == 0 and
                self.reserved == AppDescriptor.RESERVED)

    @property
    def valid(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc != 0 and self.image_size > 0 and
                self.reserved == AppDescriptor.RESERVED)


class FirmwareImage(object):
    def __init__(self, path_or_file, mode="r"):
        if getattr(path_or_file, "read", None):
            self._file = path_or_file
            self._do_close = False
        else:
            self._file = open(path_or_file, mode + "b")
            self._do_close = True

        if "r" in mode:
            self._contents = cStringIO.StringIO(self._file.read())
        else:
            self._contents = cStringIO.StringIO()
        self._do_write = False

        self._length = None
        self._descriptor_offset = None
        self._descriptor_bytes = None
        self._descriptor = None

    def __enter__(self):
        return self

    def __getattr__(self, attr):
        if attr == "write":
            self._do_write = True
        return getattr(self._contents, attr)

    def __iter__(self):
        return iter(self._contents)

    def __exit__(self, *args):
        if self._do_write:
            if getattr(self._file, "seek", None):
                self._file.seek(0)
            self._file.write(self._contents.getvalue())

        if self._do_close:
            self._file.close()

    def _write_descriptor_raw(self):
        # Seek to the appropriate location, write the serialized
        # descriptor, and seek back.
        prev_offset = self._contents.tell()
        self._contents.seek(self._descriptor_offset)
        self._contents.write(self._descriptor.pack())
        self._contents.seek(prev_offset)

    def write_descriptor(self):
        # Set the descriptor's length and CRC to the values required for
        # CRC computation
        self.app_descriptor.image_size = self.length
        self.app_descriptor.image_crc = 0

        self._write_descriptor_raw()

        # Update the descriptor's CRC based on the computed value and write
        # it out again
        self.app_descriptor.image_crc = self.crc

        self._write_descriptor_raw()

    @property
    def crc(self):
        MASK = 0xFFFFFFFFFFFFFFFF
        POLY = 0x42F0E1EBA9EA3693

        # Calculate the image CRC with the image_crc field in the app
        # descriptor zeroed out.
        crc_offset = self.app_descriptor_offset + len(AppDescriptor.SIGNATURE)
        content = bytearray(self._contents.getvalue())
        content[crc_offset:crc_offset + 8] = bytearray("\x00" * 8)

        val = MASK
        for byte in content:
            val ^= (byte << 56) & MASK
            for bit in range(8):
                if val & (1 << 63):
                    val = ((val << 1) & MASK) ^ POLY
                else:
                    val <<= 1

        return (val & MASK) ^ MASK

    @property
    def length(self):
        if not self._length:
            # Find the length of the file by seeking to the end and getting
            # the offset
            prev_offset = self._contents.tell()
            self._contents.seek(0, os.SEEK_END)
            self._length = self._contents.tell()
            self._contents.seek(prev_offset)

        return self._length

    @property
    def app_descriptor_offset(self):
        if not self._descriptor_offset:
            # Save the current position
            prev_offset = self._contents.tell()
            # Check each byte in the file to see if a valid descriptor starts
            # at that location. Slow, but not slow enough to matter.
            offset = 0
            while offset < self.length - AppDescriptor.LENGTH:
                self._contents.seek(offset)
                try:
                    # If this throws an exception, there isn't a valid
                    # descriptor at this offset
                    AppDescriptor(self._contents.read(AppDescriptor.LENGTH))
                except Exception:
                    offset += 1
                else:
                    self._descriptor_offset = offset
                    break
            # Go back to the previous position
            self._contents.seek(prev_offset)

        return self._descriptor_offset

    @property
    def app_descriptor(self):
        if not self._descriptor:
            # Save the current position
            prev_offset = self._contents.tell()
            # Jump to the descriptor adn parse it
            self._contents.seek(self.app_descriptor_offset)
            self._descriptor_bytes = self._contents.read(AppDescriptor.LENGTH)
            self._descriptor = AppDescriptor(self._descriptor_bytes)
            # Go back to the previous offset
            self._contents.seek(prev_offset)

        return self._descriptor

    @app_descriptor.setter
    def app_descriptor(self, value):
        self._descriptor = value



if __name__ == "__main__":
    log.basicConfig(format="%(asctime)-15s %(message)s", level=log.DEBUG)

    parser = OptionParser(
        usage="usage: %prog [options] CAN_DEVICE",
        version="%prog 1.0", add_help_option=False,
        description="UAVCAN management UI server")

    parser.add_option("--help", action="help",
                      help="show this help message and exit")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                      help="outputs additional log messages")
    parser.add_option("--debug", action="store_true", dest="debug",
                      help="run in debug mode (restart on code changes)")

    cmd_group = OptionGroup(parser, "Network options")
    cmd_group.add_option("-p", "--port", type="int", dest="port", default=80,
                         help="listen for HTTP requests on PORT")

    cmd_group = OptionGroup(parser, "CAN options")
    cmd_group.add_option("-n", "--node-id", dest="node_id", default=127,
                         help="run master with NODE_ID",
                         metavar="NODE_ID")
    cmd_group.add_option("-s", "--bus-speed", dest="bus_speed",
                         default=1000000, help="set CAN bus speed",
                         metavar="NODE_ID")

    cmd_group = OptionGroup(parser, "UAVCAN options")
    cmd_group.add_option("--dsdl", dest="dsdl_path", action="append",
                         metavar="PATH", help="load DSDL files from PATH")
    cmd_group.add_option("--firmware", dest="firmware", metavar="PATH",
                         help="use firmware images in PATH to update nodes")

    options, args = parser.parse_args()

    uavcan.load_dsdl(options.dsdl_path)

    ioloop = tornado.ioloop.IOLoop.instance()

    if options.firmware:
        firmware_dir = os.path.dirname(options.firmware)
    else:
        firmware_dir = None

    @gen.coroutine
    def update_device_firmware(this_node, node_id, response):
        log.debug("update_device_firmware({}, {!r})".format(node_id,
                                                            response))
        if not firmware_dir:
            log.debug("update_device_firmware(): no firmware path specified")
            return

        # If the device is in the config file, make sure it has the
        # software version listed for it in that file. If not, send a
        # BeginFirmwareUpdate with the appropriate path.
        firmware_filename = "{!s}".format(response.name.decode())
        for file in os.listdir(firmware_dir):
            if file.startswith(firmware_filename) and file.endswith("uavcan.bin"):
                firmware_filename = file 
                firmware_path = os.path.join(firmware_dir, firmware_filename)
                if os.path.exists(firmware_path):
                    with FirmwareImage(firmware_path, "rb") as f:
                        descriptor = f.app_descriptor
                        if f.app_descriptor.image_crc != \
                                response.software_version.image_crc:
                            # Version mismatch, update now.
                            request = uavcan.protocol.file.BeginFirmwareUpdate(
                                mode="request")
                            request.source_node_id = this_node.node_id
                            request.image_file_remote_path.path.encode(
                                firmware_filename)
                            response, response_transfer = \
                                yield this_node.send_request(request, node_id)
        
                            if response.error != response.ERROR_OK:
                                msg = ("[MASTER] #{0:03d} rejected "
                                       "uavcan.protocol.file.BeginFirmwareUpdate " +
                                       "with error {1:d}: {2!s}").format(
                                       node_id, response.error,
                                       response.optional_error_message.encode())
                                logging.error(msg)
                        else:
                            log.debug("update_device_firmware(): device up to date")
                else:
                    log.debug(("update_device_firmware(): no file for device " +
                               "{!s}").format(response.name.decode()))

    if len(args):
        node = uavcan.node.Node([
            # Server implementation
            (uavcan.protocol.NodeStatus, uavcan.handlers.NodeStatusHandler,
                {"new_node_callback": update_device_firmware}),
            (uavcan.protocol.dynamic_node_id.Allocation,
                uavcan.handlers.DynamicNodeIDAllocationHandler,
                {"dynamic_id_range": (2, 125)}),
            (uavcan.protocol.file.GetInfo, uavcan.handlers.FileGetInfoHandler,
                {"path": firmware_dir}),
            (uavcan.protocol.file.Read, uavcan.handlers.FileReadHandler,
                {"path": firmware_dir}),
            (uavcan.protocol.debug.LogMessage,
                uavcan.handlers.DebugLogMessageHandler),
            # CAN<->WebSocket bridge
            (uavcan.protocol.NodeStatus, MessageRelayHandler),
            (uavcan.equipment.esc.Status, MessageRelayHandler),
            (uavcan.thirdparty.thiemar.equipment.esc.Status,
                    MessageRelayHandler)
        ], node_id=int(options.node_id))
        node.listen(args[0], baudrate=int(options.bus_speed))
        #can_setpoint_schedule_timer = tornado.ioloop.PeriodicCallback(
        #    functools.partial(handle_timer, can_dev),
        #    10, io_loop=ioloop)
        #can_setpoint_schedule_timer.start()
    else:
        log.info("No CAN device specified; starting interface only")
        node = None

    app = tornado.web.Application([
            (r"/can", CANHandler, {"node": node}),
            (r"/", UI, {"environment": None}),
        ],
        debug=options.debug, gzip=True, template_path="assets",
        static_path="assets")
    http_server = tornado.httpserver.HTTPServer(app)
    http_server.listen(options.port)

    ioloop.start()
