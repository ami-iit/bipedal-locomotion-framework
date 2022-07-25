#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
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


def compute_joint_offset(sensor_bridge, joint_index, expected_values, tolerances):
    # read the joint state
    if not sensor_bridge.advance():
        raise RuntimeError('Unable to read the sensors')

    _, joints_values, _ = sensor_bridge.get_joint_positions()

    # compute the offset
    delta = joints_values[joint_index] * 180 / math.pi - expected_values[joint_index]

    tolerance = tolerances[joint_index]
    if abs(delta) < tolerances[joint_index]:
        print_info("The offset is equal to " + str(delta)
                   + " deg. The absolute value is lower than the desired tolerance "
                   + str(tolerance) + " deg.")
        print_info("The offset is set to 0 deg")
        return 0.0
    else:
        return delta


def parse_handler(file, robot_part):
    # load the parameter handler
    parameters = blf.parameters_handler.YarpParametersHandler()

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
    values = group.get_parameter_vector_float('expected_values_in_degrees')
    tolerance = group.get_parameter_vector_float('tolerance_in_degrees')

    parameters.set_parameter_vector_string('joints_list', joints)
    parameters.set_parameter_vector_string('remote_control_boards', [board])

    # populate the parameters handler with additional parameters required by the application. There is no need to
    # expose these parameter in the configuration file
    parameters.set_parameter_string('local_prefix', 'joint-offset-updater')
    parameters.set_parameter_bool('check_for_nan', False)
    parameters.set_parameter_bool('stream_joint_states', True)
    return parameters, values, joints, tolerance


def main():
    parser = argparse.ArgumentParser(description='Simple tool to semi-automatically update the calibration delta of a YARP-based robot.')
    parser.add_argument('-i', '--input', type=str, required=True, help='Path to the input xml file containing the calibration deltas')
    parser.add_argument('-o', '--output', type=str,
                        required=True, help='Path to the output xml file containing the calibration deltas')
    parser.add_argument('-p', '--part', type=str, required=True,
                        help='Name of the group will be loaded in the configuration file. For instance  left_leg or right_leg.')
    parser.add_argument('--config', type=str, required=False,
                        help='Path to the configuration file loaded by the application. '
                        'By default the blf-calibration-delta-updater uses YARP ResourceFinder '
                        'to locate a file named blf-calibration-delta-updater-options.ini',
                        default="blf-calibration-delta-updater-options.ini")
    args = parser.parse_args()

    # check if the input file exists
    if not os.path.isfile(args.input):
        raise RuntimeError('The file path ' + args.input + ' does not exist')

    output_dir = os.path.dirname(args.output)

    if output_dir and not os.path.isdir(output_dir):
        raise RuntimeError('The directory named ' + os.path.dirname(args.output)  + ' does not exist')


    # get the original offset
    tree = etree.parse(args.input)
    root = tree.getroot()
    offsets = get_offsets(root)

    # load the parameter handler
    handler, expected_values, joints_list, tolerances = parse_handler(args.config, args.part)

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
                new_offsets.append(compute_joint_offset(sensor_bridge, i, expected_values, tolerances))

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
