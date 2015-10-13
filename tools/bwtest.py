#encoding=utf-8

# Copyright (C) 2015 Thiemar Pty Ltd
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


sys.path.insert(0, "/Users/bendyer/Projects/ARM/workspace/pyuavcan/")
sys.path.insert(0, "/mnt/hgfs/workspace/pyuavcan/")
sys.path.insert(0, "/home/dev/workspace/pyuavcan/")


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


SAMPLE_RATE_HZ = 200.0


LAST_ESCSTATUS = None
LAST_HARDPOINTSTATUS = None


TEST_INDEX = 0
TEST_RUNNING = False
TEST_RESPONSE_RESULTS = collections.defaultdict(list)
TEST_POWER_RESULTS = []
NOTIFY_SOCKETS = set()


def send_all(payload):
    global NOTIFY_SOCKETS
    message = json.dumps(payload)
    for socket in NOTIFY_SOCKETS:
        socket.write_message(message)


class ESCStatusMonitor(uavcan.node.Monitor):
    def on_message(self, message):
        global LAST_ESCSTATUS
        if message.esc_index == TEST_INDEX:
            LAST_ESCSTATUS = message


class HardpointStatusMonitor(uavcan.node.Monitor):
    def on_message(self, message):
        global LAST_HARDPOINTSTATUS
        LAST_HARDPOINTSTATUS = message


class BWTest(tornado.web.RequestHandler):
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
        self.render("bwtest.html", environment=self.environment)


class WSHandler(tornado.websocket.WebSocketHandler):
    def __init__(self, *args, **kwargs):
        self.can = kwargs.pop("can", None)
        self.node = kwargs.pop("node", None)
        super(WSHandler, self).__init__(*args, **kwargs)

    def check_origin(self, origin):
        return True

    def open(self):
        self.set_nodelay(True)
        log.info("WSHandler.open()")

    def on_close(self):
        global NOTIFY_SOCKETS

        log.info("WSHandler.on_close()")
        if self in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.remove(self)

    @gen.coroutine
    def on_message(self, message):
        global TEST_RUNNING, NOTIFY_SOCKETS

        message = json.loads(message)
        log.info("WSHandler.on_message({0})".format(repr(message)))

        if self not in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.add(self)
            if TEST_RESPONSE_RESULTS:
                self.write_message(json.dumps({
                    "test": "response",
                    "data": TEST_RESPONSE_RESULTS
                }))
            if TEST_POWER_RESULTS:
                self.write_message(json.dumps({
                    "test": "power",
                    "data": TEST_POWER_RESULTS
                }))

        if not TEST_RUNNING:
            TEST_RUNNING = True
            if message.get("test") == "response":
                yield test_response(self.node)
            elif message.get("test") == "power":
                yield test_power(self.node)
            TEST_RUNNING = False


@gen.coroutine
def test_magnitude_phase(this_node, relative_amplitude, f_hz):
    global SAMPLE_RATE_HZ, LAST_ESCSTATUS, LAST_HARDPOINTSTATUS, \
           TEST_RESPONSE_RESULTS

    w = 2.0 * math.pi * f_hz
    k = 12.0 # number of periods in the sample buffer
    period_s = 1.0 / float(f_hz)
    num_samples = math.floor(k * period_s * SAMPLE_RATE_HZ)
    w_norm = w / SAMPLE_RATE_HZ
    coeff = 2.0 * math.cos(w_norm)

    # Find the magnitude and phase of the thrust estimate and the thrust
    # measurement using the Goertzel algorithm
    setpoint_q0 = setpoint_q1 = setpoint_q2 = 0.0
    est_q0 = est_q1 = est_q2 = 0.0
    meas_q0 = meas_q1 = meas_q2 = 0.0

    samples = 0.0
    tstart = time.time()
    while samples < num_samples:
        # Send setpoint
        setpoint = uavcan.equipment.esc.RawCommand()
        setpoint.cmd.append(0)
        setpoint.cmd.append(0)
        setpoint.cmd.append(int(4095.0 * (1.0 - relative_amplitude *
                                                math.cos(w_norm * samples))))
        this_node.send_message(setpoint)

        yield gen.sleep((tstart + float(samples) / SAMPLE_RATE_HZ) -
                        time.time())

        setpoint_q0 = coeff * setpoint_q1 - setpoint_q2 + \
                      LAST_ESCSTATUS.thrust_setpoint
        setpoint_q2 = setpoint_q1
        setpoint_q1 = setpoint_q0

        est_q0 = coeff * est_q1 - est_q2 + LAST_ESCSTATUS.thrust
        est_q2 = est_q1
        est_q1 = est_q0

        meas_q0 = coeff * meas_q1 - meas_q2 + \
                  (LAST_HARDPOINTSTATUS.payload_weight
                        if LAST_HARDPOINTSTATUS else 0.0)
        meas_q2 = meas_q1
        meas_q1 = meas_q0

        samples += 1.0

    # Work out magnitude as a ratio between actual and setpoint; phase is
    # relative to setpoint.
    setpoint_re = (setpoint_q1 - setpoint_q2 * math.cos(w_norm))
    setpoint_im = (setpoint_q2 * math.sin(w_norm))
    setpoint_magnitude = math.sqrt(setpoint_re**2 + setpoint_im**2)
    setpoint_phase = math.atan2(setpoint_im, setpoint_re)

    est_re = (est_q1 - est_q2 * math.cos(w_norm))
    est_im = (est_q2 * math.sin(w_norm))
    est_magnitude = math.sqrt(est_re**2 + est_im**2) / setpoint_magnitude
    est_phase = math.atan2(est_im, est_re) - setpoint_phase

    meas_re = (meas_q1 - meas_q2 * math.cos(w_norm))
    meas_im = (meas_q2 * math.sin(w_norm))
    meas_magnitude = math.sqrt(meas_re**2 + meas_im**2) / setpoint_magnitude
    meas_phase = math.atan2(meas_im, meas_re) - setpoint_phase

    # Print the result
    result = {
        "relative_amplitude": relative_amplitude,
        "f_hz": f_hz,
        "est_magnitude": est_magnitude,
        "est_phase_rad": est_phase,
        "meas_magnitude": meas_magnitude,
        "meas_phase_rad": meas_phase
    }
    TEST_RESPONSE_RESULTS[str(round(relative_amplitude * 10))].append(result)
    send_all({
        "test": "response",
        "data": TEST_RESPONSE_RESULTS
    })
    print ",".join(str(x) for x in (relative_amplitude, f_hz, est_magnitude,
                                    est_phase, meas_magnitude, meas_phase))


@gen.coroutine
def set_throttle(this_node, value, duration):
    for _ in range(int(duration / 0.05)):
        setpoint = uavcan.equipment.esc.RawCommand()
        setpoint.cmd.append(0)
        setpoint.cmd.append(0)
        setpoint.cmd.append(int(value * 8192.0))
        this_node.send_message(setpoint)
        yield gen.sleep(0.05)


@gen.coroutine
def test_response(this_node):
    global LAST_ESCSTATUS
    print "Running response test..."
    print "amplitude,f_hz,est_magnitude,est_phase_rad,meas_magnitude,meas_phase_rad"

    fstart_hz = 1.0
    fend_hz = 16.0
    k = 1.0 / 4.0  # 4 tests per octave
    fs_hz = [2**(x * k) for x in range(int(math.log(fstart_hz, 2) / k),
                                       int(math.log(fend_hz, 2) / k) + 1)]
    relative_amplitudes = [0.1, 0.2, 0.5]

    # Start-up sequence -- stay at 10% for 5 sec, then ramp from 10% up to 50%
    # in 2.5 s
    yield set_throttle(this_node, 0.1, 5.0)

    for i in range(10, 50):
        yield set_throttle(this_node, 0.01 * i, 0.05)

    # Hold at 50% until thrust stabilises
    while LAST_ESCSTATUS is None or LAST_ESCSTATUS.thrust_setpoint < 0.1 or \
            LAST_ESCSTATUS.thrust / LAST_ESCSTATUS.thrust_setpoint < 0.95:
        yield set_throttle(this_node, 0.5, 1.0)

    for relative_amplitude in relative_amplitudes:
        for f_hz in fs_hz:
            # Wait one second in between tests for the controller to stabilize
            yield set_throttle(this_node, 0.5 - relative_amplitude * 0.5, 1.0)
            yield test_magnitude_phase(this_node, relative_amplitude, f_hz)


@gen.coroutine
def test_power(this_node):
    global SAMPLE_RATE_HZ, LAST_HARDPOINTSTATUS, LAST_ESCSTATUS, \
           TEST_POWER_RESULTS
    print "Running power test..."
    print "setpoint,est_thrust_n,meas_thrust_n,power_w"

    for i in range(10, 99, 8):
        # Set throttle and wait for it to stabilise
        yield set_throttle(this_node, 0.01 * i, 1.0)

        est_thrust = 0.0
        meas_thrust = 0.0
        power = 0.0

        samples = 0.0
        tstart = time.time()
        while samples < 2.0 * SAMPLE_RATE_HZ:
            setpoint = uavcan.equipment.esc.RawCommand()
            setpoint.cmd.append(0)
            setpoint.cmd.append(0)
            setpoint.cmd.append(int(0.01 * i * 8192.0))
            this_node.send_message(setpoint)

            yield gen.sleep((tstart + float(samples) / SAMPLE_RATE_HZ) -
                            time.time())

            meas_thrust += (LAST_HARDPOINTSTATUS.payload_weight
                            if LAST_HARDPOINTSTATUS else 0.0)
            est_thrust += LAST_ESCSTATUS.thrust
            power += LAST_ESCSTATUS.power

            samples += 1.0

        est_thrust /= samples
        meas_thrust /= samples
        power /= samples

        result = {
            "throttle": i * 0.01,
            "est_thrust_n": est_thrust,
            "meas_thrust_n": meas_thrust,
            "power_w": power
        }
        TEST_POWER_RESULTS.append(result)
        send_all({
            "test": "power",
            "data": TEST_POWER_RESULTS
        })
        print ",".join(str(x) for x in (i * 0.01, est_thrust, meas_thrust,
                                        power))


@gen.coroutine
def set_integer_param(this_node, node_id, name, integer_value):
    request = uavcan.protocol.param.GetSet(mode="request")
    request.index = 0
    request.name.encode(name)
    request.value.integer_value = int(integer_value)
    (response, response_transfer), _ = yield tornado.gen.Task(
        this_node.send_request, request, node_id)
    print response
    if response.name.decode() != name or \
            response.value.integer_value != int(integer_value):
        raise ValueError(
            "set_param(): node #{} rejected parameter {} value {}".format(
            node_id, name, integer_value))


@gen.coroutine
def configure_esc(this_node, node_id):
    if node_id != 121:
        return

    # Disable ESC status messages, and send ext status at SAMPLE_RATE_HZ
    yield set_integer_param(this_node, node_id, "int_status", 50e3)
    yield set_integer_param(this_node, node_id, "int_ext_status",
                            1e6 / SAMPLE_RATE_HZ)


@gen.coroutine
def configure_hardpoint(this_node, node_id):
    # Send status messages at SAMPLE_RATE_HZ
    yield set_integer_param(this_node, node_id, "int_status",
                            1e6 / SAMPLE_RATE_HZ)


@gen.coroutine
def enumerate_device(this_node, node_id, response):
    global UAVCAN_NODE_INFO, UAVCAN_NODE_CONFIG
    log.debug("enumerate_device({}, {!r})".format(node_id, response))

    if response.name.decode() == u"com.thiemar.s2740vc-v1":
        yield configure_esc(this_node, node_id)
    elif response.name == "com.thiemar.loadsensor-v1":
        yield configure_hardpoint(this_node, node_id)


if __name__ == "__main__":
    log.basicConfig(format="%(asctime)-15s %(message)s", level=log.DEBUG)

    parser = OptionParser(
        usage="usage: %prog [options] CAN_DEVICE",
        version="%prog 1.0", add_help_option=False,
        description="UAVCAN ESC bandwidth test")

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

    options, args = parser.parse_args()

    uavcan.load_dsdl(options.dsdl_paths + ["../dsdl/thiemar"])

    ioloop = tornado.ioloop.IOLoop.instance()

    if options.log_path:
        JSON_LOG_FILE = options.log_path

    if len(args):
        node = uavcan.node.Node([
            # Server implementation
            (uavcan.protocol.NodeStatus, uavcan.monitors.NodeStatusMonitor,
                {"new_node_callback": enumerate_device}),
            (uavcan.protocol.dynamic_node_id.Allocation,
                uavcan.monitors.DynamicNodeIDServer,
                {"dynamic_id_range": (2, 125)}),
            (uavcan.thirdparty.thiemar.equipment.esc.Status, ESCStatusMonitor),
            (uavcan.equipment.hardpoint.Status, HardpointStatusMonitor),
        ], node_id=int(options.node_id))
        node.listen(args[0], baudrate=int(options.bus_speed), io_loop=ioloop)
    else:
        log.info("No CAN device specified; starting interface only")
        node = None

    app = tornado.web.Application([
            (r"/ws", WSHandler, {"node": node}),
            (r"/", BWTest, {"environment": None}),
        ],
        debug=options.debug, gzip=True, template_path="assets",
        static_path="assets")
    http_server = tornado.httpserver.HTTPServer(app)
    http_server.listen(options.port)

    ioloop.start()
