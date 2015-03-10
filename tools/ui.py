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
import can
import cgi
import json
import stat
import time
import math
import flash
import struct
import datetime
import functools
import tornado.web
import collections
import ConfigParser
import logging as log
import tornado.ioloop
import tornado.netutil
import tornado.template
from tornado import gen
import tornado.websocket
import tornado.httpserver
from optparse import OptionParser, OptionGroup


NODE_STATUS_TRANSFER_ID = 0
UAVCAN_NODESTATUS_ID = 550


CAN_COMMAND_SETPOINT_ID = 0x740
CAN_STATUS_CONTROLLER_ID = 0x730
CAN_STATUS_MEASUREMENT_ID = 0x731


NOTIFY_SOCKETS = set()
UAVCAN_NODE_UPTIME = {}
UAVCAN_NODE_CONFIG = collections.defaultdict(dict)
UAVCAN_NODE_SETPOINT_TIMER = collections.defaultdict(int)
UAVCAN_NODE_SETPOINT_SCHEDULE = collections.defaultdict(list)
UAVCAN_NODE_SETPOINT_STARTUP = collections.defaultdict(list)
UAVCAN_NODE_CONTROL_MODE = collections.defaultdict(int)
UAVCAN_NODE_MOTOR_RUNNING = collections.defaultdict(bool)
UAVCAN_NODE_MEASUREMENT_QUEUE = collections.defaultdict(collections.deque)
UAVCAN_NODE_CONTROLLER_QUEUE = collections.defaultdict(collections.deque)


SETPOINT_SCHEDULE_TIMER = 0


@tornado.gen.coroutine
def read_node_configuration(conn, node_id):
    "Send a configuration read message for each parameter, 0.01 s apart."
    param_indexes = list(v for s in flash.CAN_CONFIG_INDEXES.itervalues()
                           for k, v in s.iteritems())
    for index in param_indexes:
        command = struct.pack("<BBBBf", node_id, index, 0, 0, 0)
        conn.send(flash.CAN_COMMAND_CONFIG_ID, command)
        yield tornado.gen.Task(tornado.ioloop.IOLoop.instance().call_later,
                               0.01)


def param_index_from_param_name(name):
    """Obtain a parameter index given a parameter name in the format
    <section>_<param>."""
    section, _, name = name.partition("_")
    return flash.CAN_CONFIG_INDEXES[section][name]


def param_name_from_param_index(index):
    """Obtain a parameter name in the format <section>_<param> given a
    parameter index."""
    for section_name, params in flash.CAN_CONFIG_INDEXES.iteritems():
        for param_name, param_index in params.iteritems():
            if param_index == index:
                return section_name + "_" + param_name

    raise KeyError("Unknown parameter index " + str(index))


def send_all(message):
    global NOTIFY_SOCKETS
    for socket in NOTIFY_SOCKETS:
        socket.write_message(message)


def handle_can_message(conn, message):
    """Process an incoming CAN message."""
    global UAVCAN_NODES, UAVCAN_NODE_MEASUREMENT_QUEUE, \
           UAVCAN_NODE_CONTROLLER_QUEUE, UAVCAN_NODE_UPTIME

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
    elif message_id == CAN_STATUS_CONTROLLER_ID:
        node_id, i_d, i_q, i_setpoint = struct.unpack("<BHHH", message_data)
        i_d = can.f32_from_f16(i_d)
        i_q = can.f32_from_f16(i_q)
        i_setpoint = can.f32_from_f16(i_setpoint)
        UAVCAN_NODE_CONTROLLER_QUEUE[node_id].append({
            "i_d": i_d,
            "i_q": i_q,
            "i_setpoint": i_setpoint
        })
        if len(UAVCAN_NODE_CONTROLLER_QUEUE[node_id]) == 10:
            send_all({
                "node_id": node_id,
                "current": list(UAVCAN_NODE_CONTROLLER_QUEUE[node_id])
            })
            UAVCAN_NODE_CONTROLLER_QUEUE[node_id].clear()
    elif message_id == CAN_STATUS_MEASUREMENT_ID:
        node_id, temperature, rpm_setpoint, rpm, vbus = \
            struct.unpack("<BbhhH", message_data)
        UAVCAN_NODE_MEASUREMENT_QUEUE[node_id].append({
            "temperature": temperature,
            "rpm_setpoint": rpm_setpoint,
            "rpm": rpm,
            "vbus": can.f32_from_f16(vbus)
        })
        if len(UAVCAN_NODE_MEASUREMENT_QUEUE[node_id]) == 10:
            send_all({
                "node_id": node_id,
                "speed": list(UAVCAN_NODE_MEASUREMENT_QUEUE[node_id])
            })
            UAVCAN_NODE_MEASUREMENT_QUEUE[node_id].clear()


def send_node_status(conn):
    global NODE_STATUS_TRANSFER_ID
    message_id = can.uavcan_broadcast_id(NODE_STATUS_TRANSFER_ID,
                                         UAVCAN_NODESTATUS_ID)
    conn.send(message_id, "\x00\x00\x00\x00", True)
    NODE_STATUS_TRANSFER_ID += 1


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

        message = struct.pack("<BBhh", node_id, int(mode), int(setpoint),
                              int(setpoint))
        conn.send(CAN_COMMAND_SETPOINT_ID, message)


class CANHandler(tornado.websocket.WebSocketHandler):
    def __init__(self, *args, **kwargs):
        self.can = kwargs.pop("can", None)
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


if __name__ == "__main__":
    log.basicConfig(format="%(asctime)-15s %(message)s", level=log.DEBUG)

    parser = OptionParser(
        usage="usage: %prog [options] [CAN_DEVICE]",
        version="%prog 1.0", add_help_option=False,
        description="S2740VC management UI server")

    parser.add_option("--help", action="help",
                      help="show this help message and exit")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                      help="outputs additional log messages")
    parser.add_option("--debug", action="store_true", dest="debug",
                      help="run in debug mode (restart on code changes)")

    cmd_group = OptionGroup(parser, "Network Options")
    cmd_group.add_option("-p", "--port", type="int", dest="port", default=80,
                         help="listen for HTTP requests on PORT")

    options, args = parser.parse_args()

    asset_dir = "assets" #pkg_resources.resource_filename("ui", "assets")

    if len(args):
        can_dev = can.CAN(args[0])
        can_dev.open()
    else:
        # Stub out the CAN device
        class DummyCAN(object):
            def __init__(self):
                pass

            def send(self, *args, **kwargs):
                pass

            def add_to_ioloop(self, *args, **kwargs):
                pass

        can_dev = DummyCAN()

    app = tornado.web.Application([
            (r"/can", CANHandler, {"can": can_dev}),
            (r"/", UI, {"environment": None}),
        ],
        debug=options.debug, gzip=True, template_path=asset_dir,
        static_path=asset_dir)
    http_server = tornado.httpserver.HTTPServer(app)
    http_server.listen(options.port)
    ioloop = tornado.ioloop.IOLoop.instance()

    can_dev.add_to_ioloop(ioloop, callback=handle_can_message)
    can_node_status_timer = tornado.ioloop.PeriodicCallback(
        functools.partial(send_node_status, can_dev),
        500, io_loop=ioloop)
    can_node_status_timer.start()
    can_setpoint_schedule_timer = tornado.ioloop.PeriodicCallback(
        functools.partial(handle_timer, can_dev),
        10, io_loop=ioloop)
    can_setpoint_schedule_timer.start()

    ioloop.start()
