#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import time
import numpy as np
import datetime

import bipedal_locomotion_framework.bindings as blf
import yarp
import idyntree.swig as idyn

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

from balancing_torque_control.wbc import WBC
from balancing_torque_control.zmp import evaluate_local_zmp, evaluate_global_zmp

from datetime import timedelta


class Application:
    def __init__(self, config_file: str):
        self.first_iteration = True

        param_handler = blf.parameters_handler.YarpParametersHandler()
        if not param_handler.set_from_filename(config_file):
            raise ValueError("Unable to set the parameter handler from the file.")

        self.contact_force_threshold = param_handler.get_parameter_float(
            "contact_force_threshold"
        )

        self.dt = param_handler.get_parameter_datetime("dt")
        self.kindyn = self.build_kin_dyn(param_handler=param_handler)
        self.kindyn_with_measured = self.build_kin_dyn(param_handler=param_handler)

        # Create the polydrivers
        (
            self.poly_drivers,
            self.contact_wrenches_names,
        ) = self.build_contact_wrenches_driver(
            params_contact_wrenches=param_handler.get_group("CONTACT_WRENCHES"),
            local_prefix="balancing_torque_controller",
        )

        self.poly_drivers[
            "REMOTE_CONTROL_BOARD"
        ] = self.build_remote_control_board_driver(
            param_handler=param_handler.get_group("ROBOT_CONTROL"),
            local_prefix="balancing_torque_controller",
        )

        if not self.poly_drivers["REMOTE_CONTROL_BOARD"].is_valid():
            raise ValueError("Impossible to create the remote control board driver.")

        # just to wait that everything is in place
        blf.log().info("Sleep for two seconds. Just to be sure the interfaces are on.")
        blf.clock().sleep_for(datetime.timedelta(seconds=2))

        self.robot_control = blf.robot_interface.YarpRobotControl()
        if not self.robot_control.initialize(param_handler.get_group("ROBOT_CONTROL")):
            raise ValueError("Impossible to initialize the robot control.")

        if not self.robot_control.set_driver(
            self.poly_drivers["REMOTE_CONTROL_BOARD"].poly
        ):
            raise ValueError("Impossible to set the polydriver in the robot control.")

        # Create the sensor bridge
        self.sensor_bridge = blf.robot_interface.YarpSensorBridge()
        if not self.sensor_bridge.initialize(param_handler.get_group("SENSOR_BRIDGE")):
            raise ValueError("Impossible to initialize the sensor bridge.")

        if not self.sensor_bridge.set_drivers_list(list(self.poly_drivers.values())):
            raise ValueError("Impossible to set the sensorbridge drivers.")

        # set the kindyn computations with the robot state
        if not self.sensor_bridge.advance():
            raise ValueError("Impossible to advance the sensor bridge.")

        are_joint_ok, self.joint_positions, _ = self.sensor_bridge.get_joint_positions()
        if not are_joint_ok:
            raise ValueError("Impossible to get the joint position.")

        base_frame = param_handler.get_parameter_string("base_frame")
        base_link, self.frame_T_link = self.get_base_frame(
            base_frame, kindyn=self.kindyn
        )

        self.right_contact_frame = param_handler.get_parameter_string(
            "right_contact_frame"
        )
        self.left_contact_frame = param_handler.get_parameter_string(
            "left_contact_frame"
        )

        if not self.kindyn.setFloatingBase(base_link):
            raise ValueError("Impossible set the base link named: " + base_link)

        self.gravity = [0.0, 0.0, -blf.math.StandardAccelerationOfGravitation]
        self.base_velocity = idyn.Twist()
        self.base_velocity.zero()
        self.joint_velocities = self.joint_positions * 0
        if not self.kindyn.setRobotState(
            self.frame_T_link,
            self.joint_positions,
            self.base_velocity,
            self.joint_velocities,
            self.gravity,
        ):
            raise ValueError("Impossible set the robot state.")
        self.initial_com_position = self.kindyn.getCenterOfMassPosition().toNumPy()

        if not self.kindyn_with_measured.setFloatingBase(base_link):
            raise ValueError("Impossible set the base link named: " + base_link)
        if not self.kindyn_with_measured.setRobotState(
            self.frame_T_link,
            self.joint_positions,
            self.base_velocity,
            self.joint_velocities,
            self.gravity,
        ):
            raise ValueError("Impossible set the robot state.")

        # create and initialize the tsid
        self.tsid = WBC(
            param_handler=param_handler.get_group("TSID"),
            kindyn=self.kindyn_with_measured,
        )

        # print the solver to check if it is correct
        print(self.tsid.solver)

        I_H_r_sole = self.kindyn_with_measured.getWorldTransform(
            self.right_contact_frame
        )
        I_H_l_sole = self.kindyn_with_measured.getWorldTransform(
            self.left_contact_frame
        )

        # set the state of the right foot in tsid
        if not self.tsid.tasks["right_foot_task"].set_set_point(
            blf.conversions.to_manif_pose(
                I_H_r_sole.getRotation().toNumPy(), I_H_r_sole.getPosition().toNumPy()
            ),
            manif.SE3Tangent.Zero(),
            manif.SE3Tangent.Zero(),
        ):
            raise ValueError("Impossible to set the set point for the right foot task.")

        if not self.tsid.tasks["left_foot_task"].set_set_point(
            blf.conversions.to_manif_pose(
                I_H_l_sole.getRotation().toNumPy(), I_H_l_sole.getPosition().toNumPy()
            ),
            manif.SE3Tangent.Zero(),
            manif.SE3Tangent.Zero(),
        ):
            raise ValueError("Impossible to set the set point for the left foot task.")

        if not self.tsid.tasks["joint_regularization_task"].set_set_point(
            self.joint_positions
        ):
            raise ValueError(
                "Impossible to set the set point for the joint regularization task."
            )

        if not self.tsid.tasks["torso_task"].set_set_point(
            manif.SO3.Identity(), manif.SO3Tangent.Zero(), manif.SO3Tangent.Zero()
        ):
            raise ValueError("Impossible to set the set point for the torso task.")

        self.desired_joint_positions = self.joint_positions.copy()
        self.desired_joint_velocities = self.joint_velocities.copy()

        self.com_knots_delta_x = param_handler.get_parameter_vector_float(
            "com_knots_delta_x"
        )
        self.com_knots_delta_y = param_handler.get_parameter_vector_float(
            "com_knots_delta_y"
        )
        self.com_knots_delta_z = param_handler.get_parameter_vector_float(
            "com_knots_delta_z"
        )
        self.motion_duration = param_handler.get_parameter_datetime("motion_duration")
        self.motion_timeout = param_handler.get_parameter_datetime("motion_timeout")

        self.spline = self.create_new_spline(
            [
                self.initial_com_position
                + np.array(
                    [
                        self.com_knots_delta_x[0],
                        self.com_knots_delta_y[0],
                        self.com_knots_delta_z[0],
                    ]
                ),
                self.initial_com_position
                + np.array(
                    [
                        self.com_knots_delta_x[1],
                        self.com_knots_delta_y[1],
                        self.com_knots_delta_z[1],
                    ]
                ),
            ],
            self.motion_duration,
            self.dt,
        )

        self.index = 0
        self.knot_index = 1

        self.vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer()
        if not self.vectors_collection_server.initialize(
            param_handler.get_group("DATA_LOGGING")
        ):
            raise RuntimeError("Unable to initialize the vectors collection server")

        # populate the metadata
        self.vectors_collection_server.populate_metadata("global_zmp", ["x", "y"])
        self.vectors_collection_server.populate_metadata(
            "global_zmp_from_measured", ["x", "y"]
        )
        self.vectors_collection_server.populate_metadata("local_zmp_left", ["x", "y"])
        self.vectors_collection_server.populate_metadata("local_zmp_right", ["x", "y"])
        self.vectors_collection_server.populate_metadata(
            "com_from_desired", ["x", "y", "z"]
        )
        self.vectors_collection_server.populate_metadata(
            "com_from_measured", ["x", "y", "z"]
        )
        self.vectors_collection_server.populate_metadata(
            "desired_torque", self.robot_control.get_joint_list()
        )
        self.vectors_collection_server.finalize_metadata()

    def build_remote_control_board_driver(
        self,
        param_handler: blf.parameters_handler.IParametersHandler,
        local_prefix: str,
    ):
        param_handler.set_parameter_string("local_prefix", local_prefix)
        return blf.robot_interface.construct_remote_control_board_remapper(
            param_handler
        )

    def build_contact_wrench_driver(
        self,
        param_handler: blf.parameters_handler.IParametersHandler,
        local_prefix: str,
    ):
        param_handler.set_parameter_string("local_prefix", local_prefix)
        return blf.robot_interface.construct_generic_sensor_client(param_handler)

    def build_contact_wrenches_driver(
        self,
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
            contact_wrenches_drivers[wrench_name] = self.build_contact_wrench_driver(
                params_contact_wrenches.get_group(wrench_name), local_prefix
            )
            assert contact_wrenches_drivers[wrench_name].is_valid()
            contact_wrenches_names["left_foot"].append(
                params_contact_wrenches.get_group(wrench_name).get_parameter_string(
                    "description"
                )
            )

        for wrench_name in params_contact_wrenches.get_parameter_vector_string(
            "right_contact_wrenches_group"
        ):
            contact_wrenches_drivers[wrench_name] = self.build_contact_wrench_driver(
                params_contact_wrenches.get_group(wrench_name), local_prefix
            )
            assert contact_wrenches_drivers[wrench_name].is_valid()
            contact_wrenches_names["right_foot"].append(
                params_contact_wrenches.get_group(wrench_name).get_parameter_string(
                    "description"
                )
            )

        return contact_wrenches_drivers, contact_wrenches_names

    def build_kin_dyn(self, param_handler):
        rf = yarp.ResourceFinder()
        robot_model_path = rf.findFile("model.urdf")
        joint_list = param_handler.get_group(
            "ROBOT_CONTROL"
        ).get_parameter_vector_string("joints_list")
        ml = idyn.ModelLoader()
        ml.loadReducedModelFromFile(robot_model_path, joint_list)

        kindyn = idyn.KinDynComputations()
        kindyn.loadRobotModel(ml.model())
        return kindyn

    def get_base_frame(self, base_frame: str, kindyn: idyn.KinDynComputations):
        frame_base_index = kindyn.model().getFrameIndex(base_frame)
        link_base_index = kindyn.model().getFrameLink(frame_base_index)
        base_frame_name = kindyn.model().getLinkName(link_base_index)
        return base_frame_name, kindyn.getRelativeTransform(
            frame_base_index, link_base_index
        )

    def create_new_spline(
        self, knots_positions, motion_duration: timedelta, dt: timedelta
    ):
        com_spline = blf.planners.QuinticSpline()
        com_spline.set_initial_conditions([0, 0, 0], [0, 0, 0])
        com_spline.set_final_conditions([0, 0, 0], [0, 0, 0])
        com_spline.set_advance_time_step(dt)
        com_spline.set_knots(knots_positions, [timedelta(seconds=0), motion_duration])
        return com_spline

    def advance(self):
        # get the feedback
        if not self.sensor_bridge.advance():
            blf.log().error("Impossible to advance the sensor bridge.")
            return False

        # Get the joint position
        are_joint_ok, self.joint_positions, _ = self.sensor_bridge.get_joint_positions()
        if not are_joint_ok:
            blf.log().error("Impossible to get the joint positions.")
            return False

        # Get the joint velocity
        (
            are_joint_ok,
            self.joint_velocities,
            _,
        ) = self.sensor_bridge.get_joint_velocities()
        if not are_joint_ok:
            blf.log().error("Impossible to get the joint velocities.")
            return False

        if not self.kindyn.setRobotState(
            self.frame_T_link,
            self.desired_joint_positions,
            self.base_velocity,
            self.joint_velocities,
            self.gravity,
        ):
            blf.log().error("Impossible to set the robot state.")
            return False

        if not self.kindyn_with_measured.setRobotState(
            self.frame_T_link,
            self.joint_positions,
            self.base_velocity,
            self.joint_velocities,
            self.gravity,
        ):
            blf.log().error("Impossible to set the robot state.")
            return False

        # Advanced the CoM trajectory spline
        if not self.spline.advance():
            blf.log().error("Impossible to advance the CoM spline.")
            return False

        # solve the TSID
        com_spline_output = self.spline.get_output()
        if not self.tsid.tasks["com_task"].set_set_point(
            com_spline_output.position,
            com_spline_output.velocity,
            com_spline_output.acceleration,
        ):
            blf.log().error("Impossible to set the set point for the CoM task.")
            return False

        if not self.tsid.solver.advance() or not self.tsid.solver.is_output_valid():
            blf.log().error("Impossible to advance the TSID solver.")
            return False

        # integrate the system
        self.desired_joint_positions += (
            self.desired_joint_velocities * self.dt.total_seconds()
        )
        self.desired_joint_velocities += (
            self.tsid.solver.get_output().joint_accelerations * self.dt.total_seconds()
        )

        # if is the first iteration we switch the control mode to torque control
        if self.first_iteration:
            if not self.robot_control.set_control_mode(
                blf.robot_interface.YarpRobotControl.Torque
            ):
                blf.log().error("Impossible to set the control mode.")
                return False

            self.first_iteration = False

        # send the joint torques
        if not self.robot_control.set_references(
            self.tsid.solver.get_output().joint_torques,
            blf.robot_interface.YarpRobotControl.Torque,
        ):
            blf.log().error("Impossible to set the joint torques.")
            return False

        # send the data
        left_wrench = np.zeros(6)
        for cartesian_wrench_name in self.contact_wrenches_names["left_foot"]:
            is_ok, wrench, _ = self.sensor_bridge.get_cartesian_wrench(
                cartesian_wrench_name
            )
            if not is_ok:
                blf.log().error("Impossible to get the left wrench.")
                return False
            left_wrench += wrench

        right_wrench = np.zeros(6)
        for cartesian_wrench_name in self.contact_wrenches_names["right_foot"]:
            is_ok, wrench, _ = self.sensor_bridge.get_cartesian_wrench(
                cartesian_wrench_name
            )
            if not is_ok:
                blf.log().error("Impossible to get the right wrench.")
                return False
            right_wrench += wrench

        global_zmp = evaluate_global_zmp(
            left_wrench=left_wrench,
            right_wrench=right_wrench,
            l_sole_frame=self.left_contact_frame,
            r_sole_frame=self.right_contact_frame,
            contact_force_threshold=self.contact_force_threshold,
            kindyn=self.kindyn,
        )
        global_zmp_from_measured = evaluate_global_zmp(
            left_wrench=left_wrench,
            right_wrench=right_wrench,
            l_sole_frame=self.left_contact_frame,
            r_sole_frame=self.right_contact_frame,
            contact_force_threshold=self.contact_force_threshold,
            kindyn=self.kindyn_with_measured,
        )
        local_zmp_left, _ = evaluate_local_zmp(
            wrench=left_wrench, contact_force_threshold=self.contact_force_threshold
        )
        local_zmp_right, _ = evaluate_local_zmp(
            wrench=right_wrench, contact_force_threshold=self.contact_force_threshold
        )
        com_from_desired = self.kindyn.getCenterOfMassPosition().toNumPy()
        com_from_measured = (
            self.kindyn_with_measured.getCenterOfMassPosition().toNumPy()
        )

        self.vectors_collection_server.prepare_data()
        self.vectors_collection_server.clear_data()

        self.vectors_collection_server.populate_data("global_zmp", global_zmp)
        self.vectors_collection_server.populate_data(
            "global_zmp_from_measured", global_zmp_from_measured
        )
        self.vectors_collection_server.populate_data("local_zmp_left", local_zmp_left)
        self.vectors_collection_server.populate_data("local_zmp_right", local_zmp_right)
        self.vectors_collection_server.populate_data(
            "com_from_desired", com_from_desired
        )
        self.vectors_collection_server.populate_data(
            "com_from_measured", com_from_measured
        )
        self.vectors_collection_server.populate_data(
            "desired_torque", self.tsid.solver.get_output().joint_torques
        )

        self.vectors_collection_server.send_data()

        if self.index * self.dt >= self.motion_duration + self.motion_timeout:
            if self.knot_index + 1 >= len(self.com_knots_delta_x):
                blf.log().info("Motion completed. Closing.")
                return False

            self.spline = self.create_new_spline(
                [
                    self.initial_com_position
                    + np.array(
                        [
                            self.com_knots_delta_x[self.knot_index],
                            self.com_knots_delta_y[self.knot_index],
                            self.com_knots_delta_z[self.knot_index],
                        ]
                    ),
                    self.initial_com_position
                    + np.array(
                        [
                            self.com_knots_delta_x[self.knot_index + 1],
                            self.com_knots_delta_y[self.knot_index + 1],
                            self.com_knots_delta_z[self.knot_index + 1],
                        ]
                    ),
                ],
                self.motion_duration,
                self.dt,
            )

            self.knot_index += 1
            self.index = 0
        else:
            self.index += 1

        return True

    def __del__(self):
        # switch the control mode to position control
        if self.robot_control.is_valid():
            self.robot_control.set_control_mode(
                blf.robot_interface.YarpRobotControl.Position
            )

        blf.log().info("Closing the application.")


def main():
    # Before everything let use the YarpSink for the logger and the YarpClock as clock. These are functionalities
    # exposed by blf.
    if not blf.text_logging.LoggerBuilder.set_factory(
        blf.text_logging.YarpLoggerFactory("balancing-torque-control")
    ):
        blf.log().error("Impossible to set the logger factory.")
        return False

    if not blf.system.ClockBuilder.set_factory(blf.system.YarpClockFactory()):
        blf.log().error("Impossible to set the logger factory.")
        return False

    application = Application(config_file="blf-balancing-torque-control-options.ini")

    while True:
        tic = blf.clock().now()

        if not application.advance():
            return False

        toc = blf.clock().now()
        delta_time = toc - tic
        if delta_time < application.dt:
            blf.clock().sleep_for(application.dt - delta_time)

    return True


if __name__ == "__main__":
    network = yarp.Network()
    main()
