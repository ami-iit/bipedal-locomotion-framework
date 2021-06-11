#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

import argparse
from lxml import etree
import re
import math
import numpy as np

import bipedal_locomotion_framework.bindings as blf

def print_info(msg):
    print('\33[104m' + '[offset_updater]' + '\033[0m ' + msg)

def open_polydriver(robot_name, control_board, joints_list):
    param_handler = blf.parameters_handler.StdParametersHandler()
    param_handler.set_parameter_vector_string('joints_list', joints_list)
    param_handler.set_parameter_vector_string('remote_control_boards', [control_board])
    param_handler.set_parameter_string('robot_name', robot_name)
    param_handler.set_parameter_string('local_prefix', 'joint_offset_updater')
    param_handler.set_parameter_bool("check_for_nan", False)
    param_handler.set_parameter_bool("stream_joint_states", True)

    board = blf.robot_interface.construct_remote_control_board_remapper(param_handler)

    bridge = blf.robot_interface.YarpSensorBridge()
    bridge.initialize(param_handler)
    bridge.set_drivers_list([board])

    return board, bridge


def get_offsets(xml_root):
    deltas = []
    for node in root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    deltas = [float(x.group()) for x in re.finditer(r'[-+]?[0-9]+(?:.[0-9]+)?', param.text)]

    return deltas

def prettify(elem):
    return etree.tostring(elem, encoding="UTF-8",
                   xml_declaration=True,
                   pretty_print=True,
                   doctype='<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">')

def set_offsets(xml_root, offsets):

    for node in root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    param.text = ' '.join(format(x, "10.3f") for x in offsets)

    return xml_root

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Automatically update the calibration delta of the robot configuration file.')
    parser.add_argument('-i', '--input', type=str, required=True, help='Input xml file')
    parser.add_argument('-o', '--output', type=str, required=True, help='Output xml file')
    parser.add_argument('-b', '--board', type=str, required=True, help='Name of the control board')
    parser.add_argument('-j', '--joints', type=str, nargs='+', required=True, help='Name of the control board. The order must be the same of the one considered in the input xml')
    parser.add_argument('-r', '--robot', type=str, required=True, help='Name of the robot')

    args = parser.parse_args()

    tree = etree.parse(args.input)
    root = tree.getroot()
    offsets = get_offsets(root)

    control_board, sensor_bridge = open_polydriver(args.robot, args.board, args.joints)

    print_info("Please put the joints in zero configuration.")
    print_info("Press enter and I will show you the offsets.")
    input()

    assert sensor_bridge.advance() is True

    _, joints_values, _ = sensor_bridge.get_joint_positions()
    new_offsets = np.array(offsets) + joints_values * 180 / math.pi

    print_info("Joint values = " + str(['%.4f' % joint for joint in joints_values * 180 / math.pi]) + " deg")
    print_info("previous offset = " + str(['%.4f' % offset for offset in offsets]) + " deg")
    print_info("new offset = " + str(['%.4f' % offset for offset in new_offsets]) + " deg")

    key = ""
    while key != 'y' and key != 'n':
        print_info("Do you want to add this offset to the already existing one? [y|n]")
        key = input()
        if key == 'n' :
            print_info("Offset not changed")
        elif key == 'y' :
            root = set_offsets(root, new_offsets)

            with open(args.output, 'bw') as f:
                f.write(prettify(root))

            print_info("Offset changed")
