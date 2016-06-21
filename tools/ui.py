#encoding=utf-8

# Copyright (C) 2014-2015 Thiemar Pty Ltd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import re
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


import uavcan
import uavcan.dsdl
import uavcan.node
import uavcan.monitors
import uavcan.services
import uavcan.transport


# Py3 compatibility
try:
    long
except:
    long = int


NOTIFY_SOCKETS = set()
JSON_LOG_FILE = None


UAVCAN_NODE_INFO = {}
UAVCAN_NODE_CONFIG = {}


class UAVCANEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, uavcan.transport.CompoundValue):
            if obj.is_union:
                field = obj.union_field or obj.fields.keys()[0]
                return {field: obj.fields[field]}
            else:
                return dict((k, v) for k, v in obj.fields.items()
                            if not k.startswith("_void_"))
        elif isinstance(obj, uavcan.transport.ArrayValue):
            if getattr(obj.type.value_type, "bitlen", None) == 8 and \
                    getattr(obj.type.value_type, "kind", None) == \
                    uavcan.dsdl.parser.PrimitiveType.KIND_UNSIGNED_INT:
                try:
                    return obj.decode()
                except Exception:
                    return binascii.hexlify(obj.to_bytes())
            else:
                return list(obj)
        elif isinstance(obj, uavcan.transport.PrimitiveValue):
            return obj.value
        else:
            return json.JSONEncoder.default(self, obj)


def uavcan_value_from_obj(uavcan_value, obj):
    if isinstance(uavcan_value.type, uavcan.dsdl.parser.PrimitiveType):
        uavcan_value.value = obj
    elif isinstance(uavcan_value.type, uavcan.dsdl.parser.ArrayType):
        if isinstance(obj, list) and isinstance(uavcan_value.type.value_type,
                                            uavcan.dsdl.parser.PrimitiveType):
            del uavcan_value[:]
            for idx, obj_item in enumerate(obj):
                uavcan_value.insert(idx, obj_item)
        elif isinstance(obj, list):
            del uavcan_value[:]
            for idx, obj_item in enumerate(obj):
                uavcan_item = uavcan_value.new_item()
                uavcan_value_from_obj(uavcan_item, obj_item)
                uavcan_value.insert(idx, uavcan_item)
        elif isinstance(obj, basestring):
            try:
                uavcan_value.encode(obj)
            except Exception:
                uavcan_value.from_bytes(obj)
        else:
            raise TypeError("Expected list or string type for UAVCAN array")
    elif isinstance(uavcan_value.type, uavcan.dsdl.parser.CompoundType):
        for key, obj_value in obj.items():
            if isinstance(uavcan_value.fields[key].type,
                          uavcan.dsdl.parser.PrimitiveType):
                setattr(uavcan_value, key, obj_value)
            else:
                uavcan_field = getattr(uavcan_value, key)
                uavcan_value_from_obj(uavcan_field, obj_value)
    else:
        raise TypeError("Invalid UAVCAN object type")


def send_all(datatype, node_id, payload):
    global NOTIFY_SOCKETS, JSON_LOG_FILE

    message = json.dumps({
        "datatype": datatype,
        "node_id": node_id,
        "payload": payload
    }, cls=UAVCANEncoder)

    for socket in NOTIFY_SOCKETS:
        socket.write_message(message)

    if JSON_LOG_FILE:
        with open(JSON_LOG_FILE, "a") as f:
            f.write(message + "\n")


def format_bytes(a):
    return "".join("{:08b}".format(v)[::-1] for v in a)


class RadioRXMonitor(uavcan.node.Monitor):
    def on_message(self, message):
        log.info("RadioRXMonitor: {!s}".format(format_bytes(message.bits)))


class MessageRelayMonitor(uavcan.node.Monitor):
    def on_message(self, message):
        send_all(message.type.get_normalized_definition(),
                 self.transfer.source_node_id, message)


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

    @gen.coroutine
    def on_message(self, message):
        global NOTIFY_SOCKETS, UAVCAN_NODE_INFO, UAVCAN_NODE_CONFIG

        message = json.loads(message)

        log.info("CANHandler.on_message({0})".format(repr(message)))

        # If this is the first message from a socket, add it to the notify
        # list and send the current configuration.
        if self not in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.add(self)

            for node_id, msg in UAVCAN_NODE_INFO.iteritems():
                self.write_message(json.dumps({
                    "datatype": msg.type.get_normalized_definition(),
                    "node_id": node_id,
                    "payload": msg
                }, cls=UAVCANEncoder))

            for node_id, params in UAVCAN_NODE_CONFIG.iteritems():
                for param_name, msg in params.iteritems():
                    self.write_message(json.dumps({
                        "datatype": msg.type.get_normalized_definition(),
                        "node_id": node_id,
                        "payload": msg
                    }, cls=UAVCANEncoder))

        if "datatype" not in message:
            return

        if "node_id" in message:
            request = uavcan.TYPENAMES[message["datatype"]](mode="request")
            uavcan_value_from_obj(request, message["payload"])
            (response, response_transfer), _ = yield tornado.gen.Task(
                self.node.send_request, request, message["node_id"])
            if response:
                # Store the response if it's a parameter value
                if message["datatype"] == "uavcan.protocol.param.GetSet":
                    UAVCAN_NODE_CONFIG[message["node_id"]][response.name] = \
                        response

                # Forward the response
                send_all(response.type.get_normalized_definition(),
                         message["node_id"], response)
        else:
            payload = uavcan.TYPENAMES[message["datatype"]]()
            uavcan_value_from_obj(payload, message["payload"])
            self.node.send_message(payload)

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


def firmware_files_for_device(firmware_dir, device_name, major, minor):
    filename_regex = re.compile(
        "^{!s}-{}.{}-([0-9]+).([0-9]+).([0-9a-fA-F]+).bin$".format(
        re.escape(device_name), major, minor))

    available_files = []

    for firmware_filename in os.listdir(firmware_dir):
        match = filename_regex.match(firmware_filename)
        if not match:
            continue

        sw_major, sw_minor = map(int, match.groups()[0:2])

        firmware_path = os.path.join(firmware_dir, firmware_filename)
        mtime = os.path.getmtime(firmware_path)
        # Sort order is sw_major, sw_minor, mtime
        available_files.append((sw_major, sw_minor, mtime, firmware_path))

    # Sort descending
    return sorted(available_files, reverse=True)


@gen.coroutine
def enumerate_node_params(this_node, node_id):
    global UAVCAN_NODE_CONFIG

    param_idx = 0
    while param_idx < 8192:
        request = uavcan.protocol.param.GetSet(mode="request")
        request.index = param_idx
        (response, response_transfer), _ = yield tornado.gen.Task(
            this_node.send_request, request, node_id)
        if response and response.name:
            # Notify connected clients of the parameter information, and store
            # it so we can notify clients who connect later
            UAVCAN_NODE_CONFIG[node_id][response.name] = response
            send_all(response.type.get_normalized_definition(), node_id,
                     response)
            param_idx += 1
        else:
            break


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
    cmd_group.add_option("-l", "--log", dest="log_path", default=None,
                         help="log UAVCAN messages to PATH", metavar="PATH")

    cmd_group = OptionGroup(parser, "UAVCAN options")
    cmd_group.add_option("--dsdl", dest="dsdl_paths", action="append",
                         metavar="PATH", help="load DSDL files from PATH")
    cmd_group.add_option("--firmware", dest="firmware", metavar="PATH",
                         help="use firmware images in PATH to update nodes")

    options, args = parser.parse_args()

    uavcan.load_dsdl("../dsdl/thiemar", *(options.dsdl_paths or []))
    #uavcan.load_dsdl(*(options.dsdl_paths or []))

    ioloop = tornado.ioloop.IOLoop.instance()

    if options.firmware:
        firmware_dir = options.firmware
    else:
        firmware_dir = None

    if options.log_path:
        JSON_LOG_FILE = options.log_path

    @gen.coroutine
    def enumerate_device(this_node, node_id, response):
        global UAVCAN_NODE_INFO, UAVCAN_NODE_CONFIG
        log.debug("enumerate_device({}, {!r})".format(node_id, response))

        # Save the node info request and clear out the config
        UAVCAN_NODE_INFO[node_id] = response
        UAVCAN_NODE_CONFIG[node_id] = {}

        # Send the node info message to all connected sockets
        send_all(response.type.get_normalized_definition(), node_id, response)

        # Schedule a parameter fetch if the node is in operating mode
        if not response.status.mode:
            ioloop.add_callback(enumerate_node_params, this_node, node_id)

        # Check the supplied directory for updated firmware
        if not firmware_dir:
            log.debug("enumerate_device(): no firmware path specified")
            return

        # Search for firmware suitable for this device
        device_name = response.name.decode()
        available_files = firmware_files_for_device(
            firmware_dir, device_name, response.hardware_version.major,
            response.hardware_version.minor)

        log.debug(("enumerate_device(): found {:d} firmware file(s) " +
                   "for device '{!s}'").format(
                   len(available_files), device_name))
        for f in available_files:
            log.debug(("enumerate_device():        {!s} " +
                       "(modified {:%Y-%m-%dT%H:%M:%S})").format(
                       f[-1], datetime.datetime.fromtimestamp(f[-2])))

        # If there are files available, check the CRC of the latest version
        # against the CRC of the firmware on the node.
        if available_files:
            firmware_path = available_files[0][-1]
            with FirmwareImage(firmware_path, "rb") as f:
                if f.app_descriptor.image_crc != \
                        response.software_version.image_crc:
                    # Version mismatch, send a BeginFirmwareUpdate with the
                    # appropriate path.
                    request = uavcan.protocol.file.BeginFirmwareUpdate(
                        mode="request")
                    request.source_node_id = this_node.node_id
                    request.image_file_remote_path.path.encode(
                        os.path.basename(firmware_path))
                    (response, response_transfer), _ = yield tornado.gen.Task(
                        this_node.send_request, request, node_id)

                    if response and response.error != response.ERROR_OK:
                        msg = ("[MASTER] #{0:03d} rejected "
                               "uavcan.protocol.file.BeginFirmwareUpdate " +
                               "with error {1:d}: {2!s}").format(
                               node_id, response.error,
                               response.optional_error_message.decode())
                        log.error(msg)
                else:
                    log.debug("enumerate_device(): device up to date")

    if len(args):
        node = uavcan.node.Node([
            # Server implementation
            (uavcan.protocol.NodeStatus, uavcan.monitors.NodeStatusMonitor,
                {"new_node_callback": enumerate_device}),
            (uavcan.protocol.dynamic_node_id.Allocation,
                uavcan.monitors.DynamicNodeIDServer,
                {"dynamic_id_range": (2, 125)}),
            (uavcan.protocol.file.GetInfo, uavcan.services.FileGetInfoService,
                {"path": firmware_dir}),
            (uavcan.protocol.file.Read, uavcan.services.FileReadService,
                {"path": firmware_dir}),
            (uavcan.protocol.debug.LogMessage,
                uavcan.monitors.DebugLogMessageMonitor),
            # CAN<->WebSocket bridge
            (uavcan.protocol.NodeStatus, MessageRelayMonitor),
            (uavcan.protocol.enumeration.Indication, MessageRelayMonitor),
            (uavcan.equipment.esc.Status, MessageRelayMonitor),
            (uavcan.thirdparty.thiemar.equipment.esc.Status, MessageRelayMonitor),
            (uavcan.equipment.air_data.TrueAirspeed, MessageRelayMonitor),
            (uavcan.equipment.air_data.IndicatedAirspeed,
                MessageRelayMonitor),
            (uavcan.equipment.hardpoint.Status, MessageRelayMonitor),
            #(uavcan.thirdparty.thiemar.equipment.radio.Bitstream, RadioRXMonitor),
        ], node_id=int(options.node_id))
        node.listen(args[0], baudrate=int(options.bus_speed), io_loop=ioloop)
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
