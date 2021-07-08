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
    print('\33[104m' + '[blf-calibration-delta-updater]' + '\033[0m ' + msg)


def open_polydriver(param_handler):
    board = blf.robot_interface.construct_remote_control_board_remapper(param_handler)

    if not board.is_valid():
        raise RuntimeError('Unable to build the control board')

    bridge = blf.robot_interface.YarpSensorBridge()
    bridge.initialize(param_handler)
    bridge.set_drivers_list([board])

    return board, bridge


def get_offsets(xml_root):
    deltas = []
    for node in xml_root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    deltas = [float(x.group()) for x in re.finditer(r'[-+]?[0-9]+(?:.[0-9]+)?', param.text)]

    return deltas


def prettify(elem):
    return etree.tostring(elem,
                          encoding="UTF-8",
                          xml_declaration=True,
                          pretty_print=True,
                          doctype='<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" '
                                  '"http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">')


def set_offsets(xml_root, offsets):
    for node in xml_root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    param.text = ' '.join(format(x, "10.3f") for x in offsets)

    return xml_root


def compute_joint_offset(sensor_bridge, joint_index, expected_values):
    # read the joint state
    if not sensor_bridge.advance():
        raise RuntimeError('Unable to reda the sensors')

    _, joints_values, _ = sensor_bridge.get_joint_positions()

    # compute the offset
    delta = joints_values[joint_index] * 180 / math.pi - expected_values[joint_index]
    return delta


def parse_handler(file, robot_part):
    # load the parameter handler
    parameters = blf.parameters_handler.YarpParametersHandler()

    print("---------------------------------->" + file)

    if os.path.isfile(file):
        if not parameters.set_from_file_path(file):
            raise RuntimeError('Unable to load the configuration file')
    else:
        if not parameters.set_from_filename(file):
            raise RuntimeError('Unable to load the configuration file')

    group = parameters.get_group(robot_part)
    if group is None:
        raise RuntimeError('Unable to find the group named ' + robot_part)

    joints = group.get_parameter_vector_string('joints_list')
    board = group.get_parameter_string('control_board')
    parameters.set_parameter_vector_string('joints_list', joints)
    parameters.set_parameter_vector_string('remote_control_boards', [board])
    values = group.get_parameter_vector_float('expected_values')

    # populate the parameters handler with additional parameters required by the application. There is no need to
    # expose these parameter in the configuration file
    parameters.set_parameter_string('local_prefix', 'joint-offset-updater')
    parameters.set_parameter_bool('check_for_nan', False)
    parameters.set_parameter_bool('stream_joint_states', True)
    return parameters, values, joints


def main():
    parser = argparse.ArgumentParser(description='Automatically update the calibration delta of the robot '
                                                 'configuration file.')
    parser.add_argument('-i', '--input', type=str, required=True, help='Input xml file')
    parser.add_argument('-o', '--output', type=str, required=True, help='Output xml file')
    parser.add_argument('-p', '--part', type=str, required=True, help='Name of the part.')
    parser.add_argument('--config', type=str, required=False, help='Path or name of the configuration file',
                        default="blf-calibration-delta-updater-options.ini")
    args = parser.parse_args()

    # get the original offset
    tree = etree.parse(args.input)
    root = tree.getroot()
    offsets = get_offsets(root)

    # load the parameter handler
    handler, expected_values, joints_list = parse_handler(args.config, args.part)

    # Open the sensorbridge
    control_board, sensor_bridge = open_polydriver(handler)

    new_offsets = []
    for i, joint in enumerate(joints_list):
        key = ""
        while key != 'y' and key != 'n':
            print_info("Do you want to calibrate the joint named: " + joint + " [y|n]")
            key = input()
            if key == 'n':
                new_offsets.append(0)
            elif key == 'y':
                print_info("Please move the joint in the expected configuration. Press enter when you are ready.")
                input()
                print(expected_values)
                new_offsets.append(compute_joint_offset(sensor_bridge, i, expected_values))

    updated_offset = np.array(offsets) + np.array(new_offsets)
    print_info("Previous offsets = " + str(['%.4f' % offset for offset in offsets]) + " deg")
    print_info("New offsets = " + str(['%.4f' % offset for offset in updated_offset]) + " deg")

    # Update the configuration file
    key = ""
    while key != 'y' and key != 'n':
        print_info("Do you want to replace the new offset to the existing one? [y|n]")
        key = input()
        if key == 'n':
            print_info("Offset not changed")
        elif key == 'y':
            root = set_offsets(root, updated_offset)

            with open(args.output, 'bw') as f:
                f.write(prettify(root))

            print_info("Offset changed")


if __name__ == '__main__':
    main()
