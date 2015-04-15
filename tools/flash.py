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

import io
import can
import sys
import time
import zlib
import struct
import uavcan
import logging
import binascii
import optparse
import functools
import cStringIO
import collections
import ConfigParser


try:
    import serial
except Exception:
    print("Please install pyserial.")
    sys.exit()


try:
    import tornado
    import tornado.gen
    import tornado.ioloop
except Exception:
    print("Please install tornado.")
    sys.exit()


CAN_BOOTLOADER_REQUEST_ID = 0x720
CAN_BOOTLOADER_ANNOUNCE_ID = 0x721
CAN_BOOTLOADER_WRITE_PAGE_START_ID = 0x722
CAN_BOOTLOADER_WRITE_PAGE_DATA_ID = 0x723
CAN_BOOTLOADER_WRITE_PAGE_RESPONSE_ID = 0x724
CAN_BOOTLOADER_RESTART_ID = 0x725


CAN_BOOTLOADER_STATUS_OK = 0
CAN_BOOTLOADER_STATUS_WRONG_STATE = 1
CAN_BOOTLOADER_STATUS_WRONG_ADDRESS = 2
CAN_BOOTLOADER_STATUS_MISSING_DATA = 3
CAN_BOOTLOADER_STATUS_INVALID_CRC = 4
CAN_BOOTLOADER_STATUS_UNMODIFIED = 5


HEX_DATA_TYPE = 0
HEX_EOF_TYPE = 1
HEX_SEGMENT_ADDRESS_TYPE = 3
HEX_EXTENDED_ADDRESS_TYPE = 4
HEX_START_ADDRESS_TYPE = 5


# 64 KiB total flash, but the last 6 pages are allocated to user parameters
# the bootloader and read-only parameters, so they can't be written.
RAW_FLASH_SIZE = 64 * 1024
WRITABLE_FLASH_SIZE = RAW_FLASH_SIZE - (3 * 4 * 1024)
PAGE_SIZE = 2048
NUM_PAGES = WRITABLE_FLASH_SIZE / PAGE_SIZE


HexRecord = collections.namedtuple('HexRecord', ['length', 'address', 'rtype',
                                                 'data', 'checksum'])


def read_hex_record(idx, record):
    if not record:
        raise ValueError(("HEX record invalid: " +
                          "empty record at line {0}").format(idx))
    if record[0] != ':':
        raise ValueError(("HEX record invalid: wrong start byte '{0}' " +
                          "in record {1}: {2:r}").format(record[0], idx,
                                                         record))

    length = int(record[1:3], 16)
    address = int(record[3:7], 16)
    rtype = int(record[7:9], 16)
    data = binascii.a2b_hex(record[9:-3])
    checksum = int(record[-3:], 16)
    # Checksum is the 2s complement of the LSB of the sum of preceding bytes
    expected_checksum = \
        (~(sum(ord(b) for b in binascii.a2b_hex(record[1:-3])) & 0xFF) +1 ) & 0xFF

    if checksum != expected_checksum:
        raise ValueError(("HEX record invalid: incorrect checksum, " +
                          "expecting {0} but got {1} in record " +
                          "{2}: {3:r}").format(expected_checksum, checksum,
                                               idx, record))

    if length != len(data):
        raise ValueError(("HEX record invalid: incorrect length, " +
                          "expecting {0} but got {1} in record " +
                          "{2}: {3:r}").format(length, len(data), idx,
                                               record))

    if rtype not in (HEX_DATA_TYPE, HEX_EOF_TYPE, HEX_SEGMENT_ADDRESS_TYPE,
                     HEX_EXTENDED_ADDRESS_TYPE, HEX_START_ADDRESS_TYPE):
        raise ValueError(("HEX record invalid: unsupported data type {0} " +
                          "in record {1}: {2:r}").format(rtype, idx, record))

    return HexRecord(length, address, rtype, data, checksum)


def binary_from_hex(hexpath):
    binary = bytearray("\xFF" * WRITABLE_FLASH_SIZE)
    base_address = 0
    line = 0
    got_eof = False
    with open(hexpath, "rU") as hexfile:
        for record in hexfile:
            result = read_hex_record(line, record)
            if got_eof:
                raise ValueError(("HEX file invalid: got EOF on line "
                                  "{0}, before last record").format(line))
            elif result.rtype == HEX_EXTENDED_ADDRESS_TYPE:
                # Each section will start with a base address record, but we
                # only support single-section images.
                base_address = struct.unpack(">H", result.data)[0] << 16
            elif result.rtype == HEX_EOF_TYPE:
                # EOF record must the last in the file
                got_eof = True
            elif result.rtype == HEX_DATA_TYPE:
                # Firmware image data -- insert it into the appropriate spot
                # in the buffer. If the data starts beyond the writable
                # memory (first 56 KiB), ignore it.
                if result.address >= WRITABLE_FLASH_SIZE:
                    pass # sys.stderr.write("ignoring record: " + record)
                else:
                    binary[result.address:result.address + result.length] = \
                        result.data
            elif result.rtype in (HEX_SEGMENT_ADDRESS_TYPE,
                                  HEX_START_ADDRESS_TYPE):
                # These record types are generated for "compatibility", but
                # are unnecessary on bare-metal ARM. Ignore.
                pass
            else:
                raise ValueError()

            line += 1

    return base_address, binary


def pages_from_binary(binary):
    # Convert a binary image into indivdiual 2 KiB pages, with a page index
    # and an Adler32 checksum for each
    pages = []
    for i in xrange(len(binary) / PAGE_SIZE):
        page_data = bytearray(binary[i * PAGE_SIZE:(i + 1) * PAGE_SIZE])
        pages.append((i, page_data, zlib.adler32(str(page_data)) & 0xffffffff))

    return pages


def write_page(dev, page, timeout=5.0):
    # Write a single page of firmware
    page_idx = page[0]
    page_data = page[1]
    page_crc = page[2]
    num_words = len(page_data) / 4

    # Send page start
    page_start_msg = struct.pack("<HHL", page_idx, num_words, page_crc)
    dev.send(CAN_BOOTLOADER_WRITE_PAGE_START_ID, page_start_msg)
    # Check for errors
    result = dev.recv()
    for message in result:
        if message[0] == CAN_BOOTLOADER_WRITE_PAGE_RESPONSE_ID:
            return ord(message[1][-1])

    # Write one word at a time
    for word_idx in xrange(num_words):
        page_data_msg = struct.pack(
            "<H4B", word_idx, *page_data[word_idx * 4:(word_idx + 1) * 4])
        dev.send(CAN_BOOTLOADER_WRITE_PAGE_DATA_ID, page_data_msg)
        # Check for errors
        result = dev.recv()
        for message in result:
            if message[0] == CAN_BOOTLOADER_WRITE_PAGE_RESPONSE_ID:
                return ord(message[1][-1])

    # Wait for page confirmation
    start = time.time()
    while time.time() < start + timeout:
        result = dev.recv()
        for message in result:
            if message[0] == CAN_BOOTLOADER_WRITE_PAGE_RESPONSE_ID:
                return ord(message[1][-1])

    return -1


def update_firmware(dev, hex_path):
    print("Please connect ESC power now. Waiting for ESC startup.")

    print("Validating input HEX file at {0}.".format(hex_path))
    base_address, binary_image = binary_from_hex(hex_path)

    # Check the HEX file's base address. This must be the start of the STM32F3
    # flash address space.
    if base_address != 0x8000000:
        print(("HEX file has invalid base address {0}. The base address " +
               "must be {1}.").format(base_address, 0x8000000))

    # Convert the 56 KiB binary image to individual 2 KiB pages, each with a
    # page index and Adler32 checksum.
    pages = pages_from_binary(binary_image)

    # Clear out the buffer
    while dev.recv():
        pass

    # Send requests until we get an announce packet or a state error packet,
    # to ensure the target device is in bootloader mode.
    while True:
        dev.send(CAN_BOOTLOADER_REQUEST_ID, "")
        result = dev.recv()
        if result and any(map(
                lambda m: m[0] in (CAN_BOOTLOADER_ANNOUNCE_ID,
                                   CAN_BOOTLOADER_WRITE_PAGE_RESPONSE_ID),
                result)):
            break
        time.sleep(0.05)

    print("ESC responded to bootloader command.")

    # Clear out the buffer
    start = time.time()
    while dev.recv() or time.time() - start < 1.0:
        pass

    # Now that we've got a response, write the pages
    page_idx = 0
    while page_idx < len(pages):
        status = write_page(dev, pages[page_idx])
        # Page error handling -- if anything went wrong, retry the page.
        if status == -1:
            print("Page {0}/{1} timed out. Retrying.".format(page_idx, len(pages) - 1))
        elif status == CAN_BOOTLOADER_STATUS_OK:
            print("Page {0}/{1} complete.".format(page_idx, len(pages) - 1))
            page_idx += 1
        elif status == CAN_BOOTLOADER_STATUS_UNMODIFIED:
            print("Page {0}/{1} not modified.".format(page_idx, len(pages) - 1))
            page_idx += 1
        else:
            print("Page {0}/{1} error ({2}). Retrying.".format(page_idx, len(pages) - 1, status))

    # Page writing has finished (and must have completed successfully due to
    # the retry cycles above -- if something went wrong, the user will have
    # hit Ctrl+C so this will not be called). Restart the ESC to load the new
    # firmware.
    print("Firwmare updated. Restarting ESC.")
    dev.send(CAN_BOOTLOADER_RESTART_ID, "")


@tornado.gen.coroutine
def node_write_config(app, target_node_id, parser):
    logging.info("Writing configuration of node {0}.".format(target_node_id))

    section = "Node{0:03d}".format(target_node_id)
    if not parser.has_section(section):
        raise tornado.gen.Return()

    for param_name, param_value in parser.items(section):
        # Retrive the parameter with the current index
        param_value = float(param_value)
        logging.debug("Writing parameter {0}: {1:f}.".format(param_name, param_value))
        request = uavcan.ParamGetSetRequest()
        request.name = param_name
        request.value.float_value = param_value
        response, transfer = yield app.send_request(request, target_node_id)

        logging.info("Read parameter {0}: {1:f}.".format(
                     response.name, response.value.float_value))

        parser.set(section, response.name,
                   "{0:.4g}".format(response.value.float_value))

    raise tornado.gen.Return()


@tornado.gen.coroutine
def node_read_config(app, target_node_id, parser):
    logging.info("Reading configuration of node {0}.".format(target_node_id))

    section = "Node{0:03d}".format(target_node_id)
    parser.add_section(section)

    for index in xrange(256):
        # Retrive the parameter with the current index
        logging.debug("Retrieving parameter {0}.".format(index))
        request = uavcan.ParamGetSetRequest()
        request.index = index
        response, transfer = yield app.send_request(request, target_node_id)

        # Early exit if we hit an unknown parameter
        if response.value.float_value is None:
            break

        logging.info("Read parameter {0}, {1}: {2:f}.".format(
                     index, response.name, response.value.float_value))

        parser.set(section, response.name,
                   "{0:.4g}".format(response.value.float_value))

    raise tornado.gen.Return()


@tornado.gen.coroutine
def node_save_config(app, target_node_id):
    logging.info("Saving configuration of node {0}.".format(target_node_id))
    request = uavcan.ParamSaveEraseRequest()
    request.save = True
    response, transfer = yield app.send_request(request, target_node_id)
    raise tornado.gen.Return(response.ok)


@tornado.gen.coroutine
def node_erase_config(app, target_node_id):
    logging.info("Erasing configuration of node {0}.".format(target_node_id))
    request = uavcan.ParamSaveEraseRequest()
    request.erase = True
    response, transfer = yield app.send_request(request, target_node_id)
    raise tornado.gen.Return(response.ok)


@tornado.gen.coroutine
def node_restart(app, target_node_id):
    logging.info("Restarting node {0}.".format(target_node_id))
    request = uavcan.RestartNodeRequest()
    response, transfer = yield app.send_request(request, target_node_id)
    raise tornado.gen.Return(response.ok)


if __name__ == "__main__":
    parser = optparse.OptionParser(usage="usage: %prog [options] DEVICE")
    parser.add_option("-f", "--firmware", dest="hex_path",
                      help="flash ESC with .hex file at FILE", metavar="FILE")
    parser.add_option("-c", "--configure", dest="config_path",
                      help="configure ESC with parameters in FILE",
                      metavar="FILE")
    parser.add_option("-n", "--node-id", dest="node_id",
                      help="only flash/configure node identified by NODE_ID",
                      metavar="NODE_ID")
    parser.add_option("-r", "--read-config", dest="read_config",
                      action="store_true",
                      help="read and display device configuration")
    parser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                      help="enable verbose output")
    options, args = parser.parse_args()
    if len(args) < 1:
        parser.error("missing path to CAN device")
        sys.exit()

    logging.basicConfig(level=logging.INFO)

    logging.info("Opening CAN interface at {0}.".format(args[0]))
    app = uavcan.Application([], node_id=127)
    app.listen(args[0])

    parser = ConfigParser.SafeConfigParser()

    tornado.ioloop.IOLoop.current().run_sync(
        functools.partial(node_read_config, app, int(options.node_id), parser))

    parser.set("Node002", "uavcan_node_id", "2")

    tornado.ioloop.IOLoop.current().run_sync(
        functools.partial(node_write_config, app, int(options.node_id), parser))

    ok = tornado.ioloop.IOLoop.current().run_sync(
        functools.partial(node_save_config, app, int(options.node_id)))
    logging.info("Configuration save completed with ok={0!r}".format(ok))

    ok = tornado.ioloop.IOLoop.current().run_sync(
        functools.partial(node_restart, app, int(options.node_id)))
    logging.info("Node restart completed with ok={0!r}".format(ok))

    buf = cStringIO.StringIO()
    parser.write(buf)
    config = buf.getvalue()

    print config

