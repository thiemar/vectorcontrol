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

import io
import can
import sys
import time
import zlib
import struct
import binascii
import optparse
import cStringIO
import collections
import ConfigParser


try:
    import serial
except Exception:
    print("Please install pyserial.")
    sys.exit()


CAN_COMMAND_RESTART_MAGIC = 1860004564
CAN_COMMAND_RESTART_ID = 0x742
CAN_COMMAND_CONFIG_ID = 0x741
CAN_STATUS_CONFIG_ID = 0x732


UAVCAN_NODESTATUS_ID = 550


# Map from config file sections and items to device parameter indexes
CAN_CONFIG_INDEXES = {
    "motor": {
        "num_poles": 0,
        "current_limit": 1,
        "voltage_limit": 2,
        "rpm_max": 3,
        "rs": 4,
        "ls": 5,
        "kv": 6
    },
    "control": {
        "accel_torque_max": 7,
        "load_torque": 8,
        "accel_gain": 9,
        "accel_time": 10
    },
    "uavcan": {
        "escstatus_interval": 11,
        "node_id": 12,
        "esc_index": 13
    },
    "pwm": {
        "control_mode": 14,
        "throttle_min": 15,
        "throttle_max": 16,
        "throttle_deadband": 17,
        "control_offset": 18,
        "control_min": 19,
        "control_max": 20,
        "control_curve": 21
    }
}


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


def detect_node_ids(dev, timeout=2.0):
    print("Detecting running devices...")

    transfer_id = 0
    running_node_ids = set()

    start = time.time()
    while time.time() < start + timeout:
        # This is just a presence message to ensure ESCs are communicating
        # using UAVCAN, so it doesn't need to contain anything -- this sends
        # node uptime = 0 seconds, and node status = OK
        message_id = uavcan_broadcast_id(transfer_id, UAVCAN_NODESTATUS_ID)
        transfer_id += 1
        dev.send(message_id, "\x00\x00\x00\x00", extended=True)

        # Check for status messages from other nodes
        result = dev.recv()
        for message in result:
            if uavcan_get_data_type_id(message[0]) != UAVCAN_NODESTATUS_ID:
                continue

            node_id = uavcan_get_node_id(message[0])
            if node_id < 0x7F:
                # If found, and the node ID is valid, add it to the list
                running_node_ids.add(node_id)

        time.sleep(0.1)

    print("Found {0} devices: {1}".format(
        len(running_node_ids), ", ".join(str(x) for x in running_node_ids)))

    return running_node_ids


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


def update_config(dev, config_path, node_id):
    print("Parsing configuration file at {0}.".format(config_path))
    parser = ConfigParser.SafeConfigParser()
    parser.read(config_path)

    config_items = []

    for section in parser.sections():
        section = section.lower()
        if section not in CAN_CONFIG_INDEXES:
            print("Ignoring unknown section {0}".format(section))
            continue

        items = parser.options(section)
        for item in items:
            item = item.lower()
            if item not in CAN_CONFIG_INDEXES[section]:
                print(("Ignorning unknown item {0} in section " +
                       "{1}").format(item, section))
                continue

            # Send the configuration message
            item_id = CAN_CONFIG_INDEXES[section][item]
            if section == "pwm" and item == "control_mode":
                value = 1 if parser.get(section, item) == "torque" else 0
            else:
                value = parser.getfloat(section, item)

            command = struct.pack("<BBBBf", node_id, item_id, 1, 0, value)
            dev.send(CAN_COMMAND_CONFIG_ID, command)

            print("Setting parameter {0}_{1} to {2:.4g}.".format(section, item,
                                                                 value))

            # Wait for the reply -- the returned packet contains the current
            # value, which should normally be what we just wrote.
            result_value = None
            while result_value is None:
                result = dev.recv()
                for message in result:
                    if message[0] == CAN_STATUS_CONFIG_ID:
                        result_value = struct.unpack("<BBf", message[1])[2]

            print(("Returned value for {0}_{1} was " +
                   "{2:.4g}.").format(section, item, result_value))

            # Update the node ID if that parameter was reconfigured
            if section == "uavcan" and item == "node_id":
                node_id = int(result_value)

    # Save the configuration to flash
    print("Saving configuration.")
    command = struct.pack("<BBBBf", node_id, 0, 0, 1, 0)
    dev.send(CAN_COMMAND_CONFIG_ID, command)

    # Wait for acknowledgement
    while True:
        result = dev.recv()
        if any(map(lambda m: m[0] == CAN_STATUS_CONFIG_ID, result)):
            break

    # Once acknowledged, restart the ESC to complete the process.
    print("Configuration updated. Restarting ESC.")
    can_send_node_restart(dev, node_id)


def read_config(dev, node_id):
    print("Reading configuration of node {0}.".format(node_id))
    parser = ConfigParser.SafeConfigParser()

    # Generate reverse lookup tables so we can loop through all item indexes
    # and add the values to the relevant configuration sections
    sections_by_index = {}
    params_by_index = {}
    param_values = {}
    for section_name, params in CAN_CONFIG_INDEXES.iteritems():
        for param_name, param_index in params.iteritems():
            sections_by_index[param_index] = section_name
            params_by_index[param_index] = param_name
            param_values[param_index] = None

    for section in set(sections_by_index.values()):
        parser.add_section(section)

    for index in param_values.keys():
        # Retrive the parameter with the current index
        command = struct.pack("<BBBBf", node_id, index, 0, 0, 0)
        dev.send(CAN_COMMAND_CONFIG_ID, command)
        print("Retrieving parameter {0}.".format(index))

        # Wait for a parameter status response
        result_value = None
        while result_value is None:
            result = dev.recv()
            for message in result:
                if message[0] == CAN_STATUS_CONFIG_ID:
                    result_value = struct.unpack("<BBf", message[1])[2]

        # Add the parameter value to the map
        if index == CAN_CONFIG_INDEXES["pwm"]["control_mode"]:
            if result_value:
                result_value = "torque"
            else:
                result_value = "speed"
            parser.set(sections_by_index[index], params_by_index[index],
                       result_value)
        else:
            parser.set(sections_by_index[index], params_by_index[index],
                       "{0:.4g}".format(result_value))

    buf = cStringIO.StringIO()
    parser.write(buf)
    return buf.getvalue()


def can_send_node_restart(dev, node_id):
    print("Restarting ESC.")
    dev.send(CAN_COMMAND_RESTART_ID, chr(node_id) + "\xD4\x6A\xDD\x6E")


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

    print("Opening CAN interface at {0}.".format(args[0]))
    dev = can.CAN(args[0])
    dev.open()

    # Look for the IDs of UAVCAN nodes on the network. Doesn't really matter
    # if they're ESCs or not, because other nodes will ignore the restart
    # request.
    node_ids = detect_node_ids(dev)
    target_node_id = None

    # Ensure the node selection is unambiguous. If the user hasn't specified
    # a node ID, there must be either only one node on the network (the target
    # ESC), or the target ESC must be currently powered down.
    if len(node_ids) > 1 and not options.node_id:
        print("No node ID specified, but multiple devices are on the " +
              "network. Please use the -n/--node-id option to specify a " +
              "device to flash, or connect to only one device.")
        sys.exit()
    elif not node_ids and options.node_id:
        print(("Couldn't find node {0} on the network. Please ensure it is " +
               "running, or run this command without the -n/--node-id " +
               "option.").format(options.node_id))
        sys.exit()

    if options.node_id:
        target_node_id = options.node_id
    elif node_ids:
        target_node_id = node_ids.pop()

    # Flash the ESC's firmware if requested. If a node ID is specified, try to
    # restart it first; otherwise, if there's only one node on the network,
    # try to restart that (if the node ID wasn't specified and there's more
    # than one node, we will have already exited). If the target ESC isn't
    # powered on, the restart request will obviously do nothing, but we'll
    # start the bootloader automatically when it's powered up.
    if options.hex_path:
        if target_node_id is not None:
            can_send_node_restart(dev, target_node_id)

        print("Updating firmware. Please power the ESC up if it is not " +
              "already running.")
        update_firmware(dev, options.hex_path)

        # Look for node IDs again. This is necessary because the user may have
        # specified no node ID and started with a powered-off ESC. In this
        # scenario, only one node ID will be on the network at this point.
        if target_node_id is None:
            node_ids = detect_node_ids(dev)
            if len(node_ids) > 1:
                print("No node ID specified, but multiple devices are on " +
                      "the network. Please use the -n/--node-id option to " +
                      "specify a device to configure.")
                sys.exit()
            elif not node_ids:
                print("The device did not start up. Please power cycle it " +
                      "and try again.")
                sys.exit()

            target_node_id = node_ids.pop()
        else:
            detect_node_ids(dev)

    # If the user has requested the ESC be configured, do that now.
    if options.config_path:
        print("Updating configuration of device {0}.".format(target_node_id))
        update_config(dev, options.config_path, target_node_id)

    # If the user has asked to read the configuration of the selected device,
    # try to do that.
    if options.read_config:
        if target_node_id is None:
            print("No nodes available for reading. Please specify a node ID.")
            sys.exit()
        print(read_config(dev, target_node_id))

