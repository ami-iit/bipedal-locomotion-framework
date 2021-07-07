#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

import argparse
from lxml import etree
import re
import math
import numpy as np
import os

import bipedal_locomotion_framework.bindings as blf


def print_info(msg):
    print('\33[104m' + '[offset_updater]' + '\033[0m ' + msg)


def open_polydriver(param_handler):
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


def compute_joint_offset(joint_index, expected_values):
    # read the joint state
    assert sensor_bridge.advance() is True
    _, joints_values, _ = sensor_bridge.get_joint_positions()
    delta = joints_values[joint_index] * 180 / math.pi - expected_values[joint_index]
    return delta


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Automatically update the calibration delta of the robot configuration file.')
    parser.add_argument('-i', '--input', type=str, required=True, help='Input xml file')
    parser.add_argument('-o', '--output', type=str, required=True, help='Output xml file')
    parser.add_argument('--from', type=str, required=False, help='Path or name of the configuration file', default="blf-calibration-delta-update-options.ini")
    parser.add_argument('-p', '--part', type=str, required=True, help='name of the part.')

    args = parser.parse_args()

    # get the original offset
    tree = etree.parse(args.input)
    root = tree.getroot()
    offsets = get_offsets(root)

    # load the parameter hander
    parameterHandler = blf.parameters_handler.YarpParametersHandler()

    if (os.path.isfile(args.from)):
        assert parameterHandler.set_from_file_path(args.from) is True
    else:
        assert parameterHandler.set_from_filename(args.from) is True

    group = parameterHandler.get_group(parser.part)
    assert group is not None
    joints_list = group.get_parameter_vector_string('joints_list')
    control_board = group.get_parameter_string('control_board')
    expected_values = param_handler.get_parameter_vector_double('expected_value')
    param_handler.set_parameter_vector_string('joints_list', joints_list)
    param_handler.set_parameter_vector_string('remote_control_boards', [control_board])

    # Open the sensorbridge
    control_board, sensor_bridge = open_polydriver(parameter_handler)

    new_offsets = []

    for i, joint in enumerate(joints_list):
        key = ""
        while key != 'y' and key != 'n':
            print_info("Do you want to calibrate the joint named: " + joint + " [y|n]")
            key = input()
            if key = 'n':
                new_offsets.append(0);
            elif key == 'y' :
                print_info("Please move the joint in the expected configuration. Press enter when you are ready.")
                input()
                new_offsets.append(compute_joint_offset(i, expected_values))

    updated_offset = np.array(offsets) + np.array(new_offsets)
    print_info("previous offset = " + str(['%.4f' % offset for offset in offsets]) + " deg")
    print_info("new offset = " + str(['%.4f' % offset for offset in updated_offset]) + " deg")


    key = ""
    while key != 'y' and key != 'n':
        print_info("Do you want to add this offset to the already existing one? [y|n]")
        key = input()
        if key == 'n' :
            print_info("Offset not changed")
        elif key == 'y' :
            root = set_offsets(root, updated_offset)

            with open(args.output, 'bw') as f:
                f.write(prettify(root))

            print_info("Offset changed")
