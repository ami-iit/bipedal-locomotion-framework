#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import numpy as np
import datetime

import bipedal_locomotion_framework.bindings as blf
import yarp

from datetime import timedelta
from itertools import product

## extend the python path
from pathlib import Path
import sys

sys.path.extend(
    [
        str(
            Path(__file__).parent.resolve()
            / ".."
            / "share"
            / "BipedalLocomotionFramework"
            / "python"
        )
    ]
)


def build_remote_control_board_driver(
    param_handler: blf.parameters_handler.IParametersHandler, local_prefix: str
):
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_remote_control_board_remapper(param_handler)


def create_new_spline(knots_positions, motion_duration: timedelta, dt: timedelta):
    com_spline = blf.math.CubicSpline()
    com_spline.set_initial_conditions(
        np.zeros(len(knots_positions[0])), np.zeros(len(knots_positions[0]))
    )
    com_spline.set_final_conditions(
        np.zeros(len(knots_positions[0])), np.zeros(len(knots_positions[0]))
    )
    com_spline.set_advance_time_step(dt)
    com_spline.set_knots(knots_positions, [timedelta(seconds=0), motion_duration])
    return com_spline


def main():
    param_handler = blf.parameters_handler.YarpParametersHandler()
    if not param_handler.set_from_filename(
        "blf-joints-grid-position-tracking-options.ini"
    ):
        raise RuntimeError("Unable to load the parameters")

    # Load joints to control
    robot_control_handler = param_handler.get_group("ROBOT_CONTROL")
    joints_to_control = robot_control_handler.get_parameter_vector_string("joints_list")

    dt = param_handler.get_parameter_datetime("dt")

    poly_drivers = dict()

    poly_drivers["REMOTE_CONTROL_BOARD"] = build_remote_control_board_driver(
        param_handler=robot_control_handler,
        local_prefix="joint_position_tracking",
    )
    if not poly_drivers["REMOTE_CONTROL_BOARD"].is_valid():
        raise RuntimeError("Unable to create the remote control board driver")

    blf.log().info("Sleep for two seconds. Just to be sure the interfaces are on.")
    blf.clock().sleep_for(datetime.timedelta(seconds=2))

    robot_control = blf.robot_interface.YarpRobotControl()
    if not robot_control.initialize(robot_control_handler):
        raise RuntimeError("Unable to initialize the robot control")
    if not robot_control.set_driver(poly_drivers["REMOTE_CONTROL_BOARD"].poly):
        raise RuntimeError("Unable to set the driver for the robot control")

    # Create the sensor bridge
    sensor_bridge = blf.robot_interface.YarpSensorBridge()
    sensor_bridge_handler = param_handler.get_group("SENSOR_BRIDGE")
    if not sensor_bridge.initialize(sensor_bridge_handler):
        raise RuntimeError("Unable to initialize the sensor bridge")
    if not sensor_bridge.set_drivers_list(list(poly_drivers.values())):
        raise RuntimeError("Unable to set the drivers for the sensor bridge")

    # set the kindyn computations with the robot state
    if not sensor_bridge.advance():
        raise RuntimeError("Unable to advance the sensor bridge")

    are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
    if not are_joints_ok:
        raise RuntimeError("Unable to get the joint positions")

    # Get the desired knots
    spline_group = param_handler.get_group("SPLINE")
    knots_positions = []
    for joint in range(len(joints_to_control)):
        knots_positions.append(
            spline_group.get_parameter_vector_float(joints_to_control[joint])
        )

    knots_positions = np.array(knots_positions)

    motion_duration = spline_group.get_parameter_datetime("motion_duration")
    motion_timeout = spline_group.get_parameter_datetime("motion_timeout")

    # Create the splines by using all the possibile combinations of knots
    initial_points = []
    final_points = []
    combinations = list(
        product(range(len(knots_positions[0])), repeat=len(joints_to_control))
    )

    initial_points.append(joint_positions)
    final_point = [
        knots_positions[j][combinations[0][j]] for j in range(len(joints_to_control))
    ]
    final_points.append(final_point)

    for combination in range(len(combinations) - 1):
        initial_point = [
            knots_positions[j][combinations[combination][j]]
            for j in range(len(joints_to_control))
        ]
        final_point = [
            knots_positions[j][combinations[combination + 1][j]]
            for j in range(len(joints_to_control))
        ]

        initial_points.append(initial_point)
        final_points.append(final_point)

    vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer()
    if not vectors_collection_server.initialize(
        param_handler.get_group("DATA_LOGGING")
    ):
        raise RuntimeError("Unable to initialize the vectors collection server")

    vectors_collection_server.populate_metadata(
        "joints::desired::position",
        param_handler.get_group("ROBOT_CONTROL").get_parameter_vector_string(
            "joints_list"
        ),
    )
    vectors_collection_server.finalize_metadata()

    # switch to position direct
    robot_control.set_control_mode(blf.robot_interface.YarpRobotControl.PositionDirect)

    blf.log().info("Waiting for your input")
    blf.log().info("Press enter to start the trajectory")
    input()
    blf.log().info("Starting the trajectory")

    spline = create_new_spline(
        [initial_points[0], final_points[0]],
        motion_duration,
        dt,
    )

    index = 0
    knot_index = 1

    while True:
        tic = blf.clock().now()

        # get the feedback
        if not sensor_bridge.advance():
            raise RuntimeError("Unable to advance the sensor bridge")
        are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
        if not are_joints_ok:
            raise RuntimeError("Unable to get the joint positions")

        if index * dt >= motion_duration + motion_timeout:
            if knot_index >= len(initial_points):
                blf.log().info("Motion completed. Closing.")
                break

            # create the spline
            spline = create_new_spline(
                [initial_points[knot_index], final_points[knot_index]],
                motion_duration,
                dt,
            )

            knot_index += 1
            index = 0
        else:
            index += 1

        if not spline.advance():
            raise RuntimeError("Unable to advance the spline")

        joint_pos_des = spline.get_output().position

        # send the joint pose
        if not robot_control.set_references(
            joint_pos_des, blf.robot_interface.YarpRobotControl.PositionDirect
        ):
            raise RuntimeError("Unable to set the references")

        if not vectors_collection_server.prepare_data():
            raise RuntimeError("Unable to prepare the data")

        vectors_collection_server.clear_data()

        vectors_collection_server.populate_data(
            "joints::desired::position", joint_pos_des
        )

        vectors_collection_server.send_data()

        toc = blf.clock().now()
        delta_time = toc - tic
        if delta_time < dt:
            blf.clock().sleep_for(dt - delta_time)

    # switch to position
    robot_control.set_control_mode(blf.robot_interface.YarpRobotControl.Position)


if __name__ == "__main__":
    network = yarp.Network()
    main()
