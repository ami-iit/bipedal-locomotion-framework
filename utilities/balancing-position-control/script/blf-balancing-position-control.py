#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import time
import numpy as np
import datetime

import bipedal_locomotion_framework.bindings as blf
import yarp
import idyntree.bindings as idyn

import manifpy as manif

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

from balancing_position_control.wbc import WBC
from datetime import timedelta


def build_remote_control_board_driver(
    param_handler: blf.parameters_handler.IParametersHandler, local_prefix: str
):
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_remote_control_board_remapper(param_handler)


def build_contact_wrench_driver(
    param_handler: blf.parameters_handler.IParametersHandler, local_prefix: str
):
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_generic_sensor_client(param_handler)


def build_contact_wrenches_driver(
    params_contact_wrenches: blf.parameters_handler.IParametersHandler,
    local_prefix: str,
):
    # build contact wrenches polydrivers
    contact_wrenches_drivers = dict()
    contact_wrenches_names = dict()
    contact_wrenches_names["left_foot"] = []
    contact_wrenches_names["right_foot"] = []

    for wrench_name in params_contact_wrenches.get_parameter_vector_string(
        "left_contact_wrenches_group"
    ):
        contact_wrenches_drivers[wrench_name] = build_contact_wrench_driver(
            params_contact_wrenches.get_group(wrench_name), local_prefix
        )
        if not contact_wrenches_drivers[wrench_name].is_valid():
            raise RuntimeError(
                "Unable to create the contact wrench driver for " + wrench_name
            )

        contact_wrenches_names["left_foot"].append(
            params_contact_wrenches.get_group(wrench_name).get_parameter_string(
                "description"
            )
        )

    for wrench_name in params_contact_wrenches.get_parameter_vector_string(
        "right_contact_wrenches_group"
    ):
        contact_wrenches_drivers[wrench_name] = build_contact_wrench_driver(
            params_contact_wrenches.get_group(wrench_name), local_prefix
        )
        if not contact_wrenches_drivers[wrench_name].is_valid():
            raise RuntimeError(
                "Unable to create the contact wrench driver for " + wrench_name
            )

        contact_wrenches_names["right_foot"].append(
            params_contact_wrenches.get_group(wrench_name).get_parameter_string(
                "description"
            )
        )

    return contact_wrenches_drivers, contact_wrenches_names


def build_kin_dyn(param_handler):
    rf = yarp.ResourceFinder()
    robot_model_path = rf.findFile("model.urdf")
    joint_list = param_handler.get_group("ROBOT_CONTROL").get_parameter_vector_string(
        "joints_list"
    )
    ml = idyn.ModelLoader()
    ml.loadReducedModelFromFile(robot_model_path, joint_list)

    kindyn = idyn.KinDynComputations()
    kindyn.loadRobotModel(ml.model())
    return kindyn


def get_base_frame(base_frame: str, kindyn: idyn.KinDynComputations):
    frame_base_index = kindyn.model().getFrameIndex(base_frame)
    link_base_index = kindyn.model().getFrameLink(frame_base_index)
    base_frame_name = kindyn.model().getLinkName(link_base_index)
    return base_frame_name, kindyn.getRelativeTransform(
        frame_base_index, link_base_index
    )


def create_new_spline(knots_positions, motion_duration: timedelta, dt: timedelta):
    com_spline = blf.math.QuinticSpline()
    com_spline.set_initial_conditions([0, 0, 0], [0, 0, 0])
    com_spline.set_final_conditions([0, 0, 0], [0, 0, 0])
    com_spline.set_advance_time_step(dt)
    com_spline.set_knots(knots_positions, [timedelta(seconds=0), motion_duration])
    return com_spline


def main():
    # Before everything let use the YarpSink for the logger and the YarpClock as clock. These are functionalities
    # exposed by blf.
    if not blf.text_logging.LoggerBuilder.set_factory(
        blf.text_logging.YarpLoggerFactory("balancing-position-control")
    ):
        raise RuntimeError("Unable to set the logger factory")
    if not blf.system.ClockBuilder.set_factory(blf.system.YarpClockFactory()):
        raise RuntimeError("Unable to set the clock factory")

    param_handler = blf.parameters_handler.YarpParametersHandler()
    if not param_handler.set_from_filename(
        "blf-balancing-position-control-options.ini"
    ):
        raise RuntimeError("Unable to load the parameters")

    contact_force_threshold = param_handler.get_parameter_float(
        "contact_force_threshold"
    )

    dt = param_handler.get_parameter_datetime("dt")

    kindyn = build_kin_dyn(param_handler=param_handler)
    kindyn_with_measured = build_kin_dyn(param_handler=param_handler)

    # Create the polydrivers
    poly_drivers, contact_wrenches_names = build_contact_wrenches_driver(
        params_contact_wrenches=param_handler.get_group("CONTACT_WRENCHES"),
        local_prefix="balancing_controller",
    )

    poly_drivers["REMOTE_CONTROL_BOARD"] = build_remote_control_board_driver(
        param_handler=param_handler.get_group("ROBOT_CONTROL"),
        local_prefix="balancing_controller",
    )
    if not poly_drivers["REMOTE_CONTROL_BOARD"].is_valid():
        raise RuntimeError("Unable to create the remote control board driver")

    # just to wait that everything is in place
    blf.log().info("Sleep for two seconds. Just to be sure the interfaces are on.")
    blf.clock().sleep_for(datetime.timedelta(seconds=2))

    robot_control = blf.robot_interface.YarpRobotControl()
    if not robot_control.initialize(param_handler.get_group("ROBOT_CONTROL")):
        raise RuntimeError("Unable to initialize the robot control")
    if not robot_control.set_driver(poly_drivers["REMOTE_CONTROL_BOARD"].poly):
        raise RuntimeError("Unable to set the driver for the robot control")

    # Create the sensor bridge
    sensor_bridge = blf.robot_interface.YarpSensorBridge()
    if not sensor_bridge.initialize(param_handler.get_group("SENSOR_BRIDGE")):
        raise RuntimeError("Unable to initialize the sensor bridge")
    if not sensor_bridge.set_drivers_list(list(poly_drivers.values())):
        raise RuntimeError("Unable to set the drivers for the sensor bridge")

    # set the kindyn computations with the robot state
    if not sensor_bridge.advance():
        raise RuntimeError("Unable to advance the sensor bridge")

    are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
    if not are_joints_ok:
        raise RuntimeError("Unable to get the joint positions")

    base_frame = param_handler.get_parameter_string("base_frame")
    base_link, frame_T_link = get_base_frame(base_frame, kindyn=kindyn)

    right_contact_frame = param_handler.get_parameter_string("right_contact_frame")
    left_contact_frame = param_handler.get_parameter_string("left_contact_frame")

    if not kindyn.setFloatingBase(base_link):
        raise RuntimeError("Unable to set the floating base")

    # set the robot state for the kindyn computations object
    gravity = [0.0, 0.0, -blf.math.StandardAccelerationOfGravitation]
    base_velocity = idyn.Twist()
    base_velocity.zero()
    desired_joint_velocities = joint_positions * 0
    joint_velocities = desired_joint_velocities.copy()

    if not kindyn.setRobotState(
        frame_T_link, joint_positions, base_velocity, joint_velocities, gravity
    ):
        raise RuntimeError("Unable to set the robot state")
    initial_com_position = kindyn.getCenterOfMassPosition().toNumPy()

    if not kindyn_with_measured.setFloatingBase(base_link):
        raise RuntimeError("Unable to set the floating base")
    if not kindyn_with_measured.setRobotState(
        frame_T_link, joint_positions, base_velocity, joint_velocities, gravity
    ):
        raise RuntimeError("Unable to set the robot state")

    global_cop_evaluator = blf.contacts.GlobalCoPEvaluator()
    if not global_cop_evaluator.initialize(
        param_handler.get_group("GLOBAL_COP_EVALUATOR")
    ):
        raise RuntimeError("Unable to initialize the global cop evaluator")

    close_loop_with_zmp = param_handler.get_parameter_bool("close_loop_with_zmp")

    # create zmp-com controller if close_loop_with_zmp is true
    com_zmp_controller = blf.simplified_model_controllers.CoMZMPController()
    if close_loop_with_zmp:
        if not com_zmp_controller.initialize(
            param_handler.get_group("COM_ZMP_CONTROLLER")
        ):
            raise RuntimeError("Unable to initialize the zmp-com controller")

    # create and initialize the IK
    ik = WBC(param_handler=param_handler.get_group("IK"), kindyn=kindyn)
    I_H_r_sole = kindyn.getWorldTransform(right_contact_frame)
    I_H_l_sole = kindyn.getWorldTransform(left_contact_frame)
    if not ik.tasks["right_foot_task"].set_set_point(
        blf.conversions.to_manif_pose(
            I_H_r_sole.getRotation().toNumPy(), I_H_r_sole.getPosition().toNumPy()
        )
    ):
        raise RuntimeError("Unable to set the set point for the right foot task")
    if not ik.tasks["left_foot_task"].set_set_point(
        blf.conversions.to_manif_pose(
            I_H_l_sole.getRotation().toNumPy(), I_H_l_sole.getPosition().toNumPy()
        )
    ):
        raise RuntimeError("Unable to set the set point for the left foot task")
    if not ik.tasks["joint_regularization_task"].set_set_point(joint_positions):
        raise RuntimeError(
            "Unable to set the set point for the joint regularization task"
        )
    if not ik.tasks["torso_task"].set_set_point(manif.SO3.Identity()):
        raise RuntimeError("Unable to set the set point for the torso task")

    desired_joint_positions = joint_positions.copy()

    com_knots_delta_x = param_handler.get_parameter_vector_float("com_knots_delta_x")
    com_knots_delta_y = param_handler.get_parameter_vector_float("com_knots_delta_y")
    com_knots_delta_z = param_handler.get_parameter_vector_float("com_knots_delta_z")
    motion_duration = param_handler.get_parameter_datetime("motion_duration")
    motion_timeout = param_handler.get_parameter_datetime("motion_timeout")

    spline = create_new_spline(
        [
            initial_com_position
            + np.array(
                [com_knots_delta_x[0], com_knots_delta_y[0], com_knots_delta_z[0]]
            ),
            initial_com_position
            + np.array(
                [com_knots_delta_x[1], com_knots_delta_y[1], com_knots_delta_z[1]]
            ),
        ],
        motion_duration,
        dt,
    )

    index = 0
    knot_index = 1

    lipm_omega_square = (
        blf.math.StandardAccelerationOfGravitation / initial_com_position[2]
    )

    desired_com_position = initial_com_position.copy()

    vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer()
    if not vectors_collection_server.initialize(
        param_handler.get_group("DATA_LOGGING")
    ):
        raise RuntimeError("Unable to initialize the vectors collection server")

    vectors_collection_server.populate_metadata("zmp::desired_planner", ["x", "y", "z"])
    vectors_collection_server.populate_metadata(
        "zmp::measured::global::with_joint_desired", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "zmp::measured::global::with_joint_measured", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "zmp::measured::local::left", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "zmp::measured::local::right", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::measured::with_joint_desired", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::measured::with_joint_measured", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::planned::position", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::planned::velocity", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::planned::acceleration", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::com_zmp::position", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "com::com_zmp::velocity", ["x", "y", "z"]
    )
    vectors_collection_server.populate_metadata(
        "joints::desired::position",
        param_handler.get_group("ROBOT_CONTROL").get_parameter_vector_string(
            "joints_list"
        ),
    )
    vectors_collection_server.finalize_metadata()

    # switch to position direct
    robot_control.set_control_mode(blf.robot_interface.YarpRobotControl.PositionDirect)

    blf.log().info("Starting the balancing controller. Waiting for your input")
    blf.log().info("Press enter to start the balancing controller")
    input()
    blf.log().info("Starting the balancing controller")

    while True:
        tic = blf.clock().now()

        # get the feedback
        if not sensor_bridge.advance():
            raise RuntimeError("Unable to advance the sensor bridge")
        are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
        if not are_joints_ok:
            raise RuntimeError("Unable to get the joint positions")

        are_joints_ok, joint_velocities, _ = sensor_bridge.get_joint_velocities()
        if not are_joints_ok:
            raise RuntimeError("Unable to get the joint velocities")

        if not kindyn.setRobotState(
            frame_T_link,
            desired_joint_positions,
            base_velocity,
            desired_joint_velocities,
            gravity,
        ):
            raise RuntimeError("Unable to set the robot state")
        if not kindyn_with_measured.setRobotState(
            frame_T_link, joint_positions, base_velocity, joint_velocities, gravity
        ):
            raise RuntimeError("Unable to set the robot state")

        left_wrench = np.zeros(6)
        for cartesian_wrench_name in contact_wrenches_names["left_foot"]:
            _, wrench, _ = sensor_bridge.get_cartesian_wrench(cartesian_wrench_name)
            left_wrench += wrench

        right_wrench = np.zeros(6)
        for cartesian_wrench_name in contact_wrenches_names["right_foot"]:
            _, wrench, _ = sensor_bridge.get_cartesian_wrench(cartesian_wrench_name)
            right_wrench += wrench

        # evaluate the global CoP using the desired joint state
        left_contact = blf.contacts.ContactWrench()
        left_contact.wrench = left_wrench
        left_contact.pose = blf.conversions.to_manif_pose(
            kindyn.getWorldTransform(left_contact_frame)
        )
        right_contact = blf.contacts.ContactWrench()
        right_contact.wrench = right_wrench
        right_contact.pose = blf.conversions.to_manif_pose(
            kindyn.getWorldTransform(right_contact_frame)
        )

        if not global_cop_evaluator.set_input([left_contact, right_contact]):
            raise RuntimeError("Unable to set the input for the global cop evaluator")
        if not global_cop_evaluator.advance():
            raise RuntimeError("Unable to advance the global cop evaluator")
        global_zmp = global_cop_evaluator.get_output()

        # evaluate the global CoP using the measured joint state
        left_contact = blf.contacts.ContactWrench()
        left_contact.wrench = left_wrench
        left_contact.pose = blf.conversions.to_manif_pose(
            kindyn_with_measured.getWorldTransform(left_contact_frame)
        )
        right_contact = blf.contacts.ContactWrench()
        right_contact.wrench = right_wrench
        right_contact.pose = blf.conversions.to_manif_pose(
            kindyn_with_measured.getWorldTransform(right_contact_frame)
        )
        if not global_cop_evaluator.set_input([left_contact, right_contact]):
            raise RuntimeError("Unable to set the input for the global cop evaluator")
        if not global_cop_evaluator.advance():
            raise RuntimeError("Unable to advance the global cop evaluator")
        global_zmp_from_measured = global_cop_evaluator.get_output()

        # use the CoM-ZMP controller
        if not spline.advance():
            raise RuntimeError("Unable to advance the spline")

        com_spline_output = spline.get_output()
        # evaluate the desired ZMP using the LIP model
        # ddx_com = omega^2 * (x_com - x_zmp)
        desired_zmp = (
            com_spline_output.position[:2]
            - com_spline_output.acceleration[:2] / lipm_omega_square
        )
        desired_zmp = np.append(desired_zmp, 0.0)

        # set the desired ZMP and the feedback if close_loop_with_zmp is true
        if close_loop_with_zmp:
            com_zmp_controller.set_set_point(
                com_spline_output.velocity[:2],
                com_spline_output.position[:2],
                desired_zmp[:2],
            )
            com_zmp_controller.set_feedback(
                kindyn_with_measured.getCenterOfMassPosition().toNumPy()[:2],
                global_zmp_from_measured[:2],
                0,
            )
            if not com_zmp_controller.advance():
                raise RuntimeError("Unable to advance the CoM-ZMP controller")

        # evaluate the desired CoM position
        if close_loop_with_zmp:
            desired_com_velocity = np.append(
                com_zmp_controller.get_output(), com_spline_output.velocity[2]
            )
            desired_com_position[0:2] += (
                com_zmp_controller.get_output() * dt.total_seconds()
            )
            desired_com_position[2] = com_spline_output.position[2]
        else:
            desired_com_velocity = com_spline_output.velocity
            desired_com_position = com_spline_output.position

        # solve the IK
        if not ik.tasks["com_task"].set_set_point(
            desired_com_position, desired_com_velocity
        ):
            raise RuntimeError("Unable to set the set point for the com task")
        if not ik.solver.advance():
            raise RuntimeError("Unable to advance the solver")
        if not ik.solver.is_output_valid():
            raise RuntimeError("The solver output is not valid")

        # integrate the system
        desired_joint_positions += (
            ik.solver.get_output().joint_velocity * dt.total_seconds()
        )
        desired_joint_velocities = ik.solver.get_output().joint_velocity

        # send the joint pose
        if not robot_control.set_references(
            desired_joint_positions,
            blf.robot_interface.YarpRobotControl.PositionDirect,
            joint_positions,
        ):
            raise RuntimeError("Unable to set the references")

        left_wrench = blf.math.Wrenchd(left_wrench)
        local_zmp_left = left_wrench.get_local_cop()
        right_wrench = blf.math.Wrenchd(right_wrench)
        local_zmp_right = right_wrench.get_local_cop()

        com_from_desired = kindyn.getCenterOfMassPosition().toNumPy()
        com_from_measured = kindyn_with_measured.getCenterOfMassPosition().toNumPy()

        vectors_collection_server.prepare_data()
        vectors_collection_server.clear_data()

        vectors_collection_server.populate_data("zmp::desired_planner", desired_zmp)
        vectors_collection_server.populate_data(
            "zmp::measured::global::with_joint_desired", global_zmp
        )
        vectors_collection_server.populate_data(
            "zmp::measured::global::with_joint_measured", global_zmp_from_measured
        )
        vectors_collection_server.populate_data(
            "zmp::measured::local::left", local_zmp_left
        )
        vectors_collection_server.populate_data(
            "zmp::measured::local::right", local_zmp_right
        )
        vectors_collection_server.populate_data(
            "com::measured::with_joint_desired", com_from_desired
        )
        vectors_collection_server.populate_data(
            "com::measured::with_joint_measured", com_from_measured
        )
        vectors_collection_server.populate_data(
            "com::planned::position", com_spline_output.position
        )
        vectors_collection_server.populate_data(
            "com::planned::velocity", com_spline_output.velocity
        )
        vectors_collection_server.populate_data(
            "com::planned::acceleration", com_spline_output.acceleration
        )
        vectors_collection_server.populate_data(
            "com::com_zmp::position", desired_com_position
        )
        vectors_collection_server.populate_data(
            "com::com_zmp::velocity", desired_com_velocity
        )
        vectors_collection_server.populate_data(
            "joints::desired::position", desired_joint_positions
        )

        vectors_collection_server.send_data()

        if index * dt >= motion_duration + motion_timeout:
            if knot_index + 1 >= len(com_knots_delta_x):
                blf.log().info("Motion completed. Closing.")
                break

            spline = create_new_spline(
                [
                    initial_com_position
                    + np.array(
                        [
                            com_knots_delta_x[knot_index],
                            com_knots_delta_y[knot_index],
                            com_knots_delta_z[knot_index],
                        ]
                    ),
                    initial_com_position
                    + np.array(
                        [
                            com_knots_delta_x[knot_index + 1],
                            com_knots_delta_y[knot_index + 1],
                            com_knots_delta_z[knot_index + 1],
                        ]
                    ),
                ],
                motion_duration,
                dt,
            )

            knot_index += 1
            index = 0
        else:
            index += 1

        toc = blf.clock().now()
        delta_time = toc - tic
        if delta_time < dt:
            blf.clock().sleep_for(dt - delta_time)


if __name__ == "__main__":
    network = yarp.Network()
    main()
