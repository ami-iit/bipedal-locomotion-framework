#!/usr/bin/env python3

# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import argparse
import datetime
import os
import signal
import sys
from abc import ABC, abstractmethod
from typing import Callable, List, Optional, Type

import bipedal_locomotion_framework as blf
import numpy as np
import numpy.typing as npt
import yarp

logPrefix = "[MotorCurrentTrackingApplication]"

ParamHandler = Type[blf.parameters_handler.YarpParametersHandler]
RobotControl = Type[blf.robot_interface.YarpRobotControl]
SensorBridge = Type[blf.robot_interface.YarpSensorBridge]
PolyDriver = Type[blf.robot_interface.PolyDriver]


class MotorParameters(ABC):
    # k_tau[A/Nm] includes the gear ratio
    k_tau = dict()
    # max_safety_current[A] limits the motor current to avoid damages
    max_safety_current = dict()

    @staticmethod
    def from_parameter_handler(motor_param_handler: ParamHandler) -> None:

        joints_list = motor_param_handler.get_parameter_vector_string("joints_list")
        k_tau = motor_param_handler.get_parameter_vector_float("k_tau")
        max_safety_current = motor_param_handler.get_parameter_vector_float(
            "max_safety_current"
        )

        if len(joints_list) != len(k_tau):
            raise ValueError(
                "{} The number of joints must be equal to the size of the k_tau".format(
                    logPrefix
                )
            )

        if len(joints_list) != len(max_safety_current):
            raise ValueError(
                "{} The number of joints must be equal to the size of the max_current".format(
                    logPrefix
                )
            )

        MotorParameters.k_tau = dict(zip(joints_list, k_tau))
        MotorParameters.max_safety_current = dict(zip(joints_list, max_safety_current))


class Trajectory(ABC):
    @abstractmethod
    def generate(self, *args, **kwargs):
        pass


class SinusoidTrajectoryGenerator(Trajectory):
    def __init__(
        self,
        min_delta_current: npt.NDArray[np.float_],
        max_delta_current: npt.NDArray[np.float_],
        delta_current_increment: npt.NDArray[np.float_],
        min_frequency: npt.NDArray[np.float_],
        max_frequency: npt.NDArray[np.float_],
        frequency_decrement: npt.NDArray[np.float_],
    ):
        self.min_delta_current = min_delta_current
        self.max_delta_current = max_delta_current
        self.delta_current_increment = delta_current_increment
        self.min_frequency = min_frequency
        self.max_frequency = max_frequency
        self.frequency_decrement = frequency_decrement
        self.joint_list = []
        self.trajectory = np.array([])

    def from_parameter_handler(
        param_handler: ParamHandler,
    ) -> "SinusoidTrajectoryGenerator":

        min_delta_current = param_handler.get_parameter_vector_float(
            "min_delta_current"
        )
        max_delta_current = param_handler.get_parameter_vector_float(
            "max_delta_current"
        )
        delta_current_increment = param_handler.get_parameter_vector_float(
            "delta_current_increment"
        )
        min_frequency = param_handler.get_parameter_vector_float("min_frequency")
        max_frequency = param_handler.get_parameter_vector_float("max_frequency")
        frequency_decrement = param_handler.get_parameter_vector_float(
            "frequency_decrement"
        )

        return SinusoidTrajectoryGenerator(
            min_delta_current=min_delta_current,
            max_delta_current=max_delta_current,
            delta_current_increment=delta_current_increment,
            min_frequency=min_frequency,
            max_frequency=max_frequency,
            frequency_decrement=frequency_decrement,
        )

    def set_joint_list(self, joint_list: List[str]):
        self.joint_list = joint_list

    def generate(
        self,
        dt: float,
        initial_current: float,
        joint_index: Optional[int] = None,
        opposite_direction: Optional[bool] = False,
    ) -> List[float]:

        # Check if joint list is set
        if not self.joint_list:
            raise ValueError(
                "{} Joint list must be set before generating the trajectory".format(
                    logPrefix
                )
            )

        # Check if joint index has to be specified
        if len(self.joint_list) > 1 and (joint_index is None):
            raise ValueError(
                "{} Joint index must be specified when more than one joint is controlled".format(
                    logPrefix
                )
            )

        # Set default joint index, if not specified
        if joint_index is None:
            joint_index = 0

        A = self.min_delta_current[joint_index]
        f_in = self.max_frequency[joint_index]
        f_end = self.min_frequency[joint_index]
        delta_f = -self.frequency_decrement[joint_index]
        delta_current_increment = self.delta_current_increment[joint_index]

        # Generate the trajectory
        trajectory = []

        while np.abs(A) <= np.abs(self.max_delta_current[joint_index]):

            for f in np.arange(f_in, f_end, delta_f):
                t_max = 2.0 / f
                t = np.arange(0, t_max, dt)
                trajectory.extend(initial_current + A * np.sin(2 * np.pi * f * t))

            A += delta_current_increment

        return trajectory

    def create_starting_points(
        self,
        number_of_starting_points: int,
        number_of_joints: int,
        lower_limits: npt.NDArray[np.float_],
        upper_limits: npt.NDArray[np.float_],
        safety_threshold: Optional[float] = 0.0,
    ) -> npt.NDArray[np.float_]:

        if (
            number_of_joints != lower_limits.size
            or number_of_joints != upper_limits.size
        ):
            raise ValueError(
                "{} The number of joints must be equal to the size of the lower and upper limits".format(
                    logPrefix
                )
            )

        starting_points = np.zeros((number_of_starting_points, number_of_joints))
        for joint_index in range(number_of_joints):
            tmp = np.linspace(
                lower_limits[joint_index] + safety_threshold,
                upper_limits[joint_index] - safety_threshold,
                number_of_starting_points + 2,
            )
            starting_points[:, joint_index] = tmp[1:-1]

        return starting_points


class RampTrajectoryGenerator(Trajectory):
    def __init__(
        self,
        max_current_increment: npt.NDArray[np.float_],
        speed_in: npt.NDArray[np.float_],
        speed_end: npt.NDArray[np.float_],
        speed_increment: npt.NDArray[np.float_],
    ):
        self.max_current_increment = max_current_increment
        self.speed_in = speed_in
        self.speed = speed_in.copy()
        self.speed_end = speed_end
        self.speed_increment = speed_increment
        self.joint_list = []
        self.trajectory = np.array([])

    def from_parameter_handler(
        param_handler: ParamHandler,
    ) -> "RampTrajectoryGenerator":

        max_current_increment = param_handler.get_parameter_vector_float(
            "max_current_increment"
        )
        speed_in = param_handler.get_parameter_vector_float("initial_speed")
        speed_end = param_handler.get_parameter_vector_float("final_speed")
        speed_increment = param_handler.get_parameter_vector_float("speed_increment")

        return RampTrajectoryGenerator(
            max_current_increment=max_current_increment,
            speed_in=speed_in,
            speed_end=speed_end,
            speed_increment=speed_increment,
        )

    def set_joint_list(self, joint_list: List[str]):
        self.joint_list = joint_list

    def reset_velocity(self, joint_index: Optional[int] = None):

        # Set default joint index, if not specified
        if joint_index is None:
            joint_index = 0

        self.speed[joint_index] = self.speed_in[joint_index]

    def increment_velocity(self, joint_index: Optional[int] = None):

        # Set default joint index, if not specified
        if joint_index is None:
            joint_index = 0

        self.speed[joint_index] = (
            self.speed[joint_index] + self.speed_increment[joint_index]
        )

    def generate(
        self,
        dt: float,
        initial_current: float,
        joint_index: Optional[int] = None,
        opposite_direction: Optional[bool] = False,
    ) -> List[float]:

        # Check if joint list is set
        if not self.joint_list:
            raise ValueError(
                "{} Joint list must be set before generating the trajectory".format(
                    logPrefix
                )
            )

        # Check if joint index has to be specified
        if len(self.joint_list) > 1 and (joint_index is None):
            raise ValueError(
                "{} Joint index must be specified when more than one joint is controlled".format(
                    logPrefix
                )
            )

        # Set default joint index, if not specified
        if joint_index is None:
            joint_index = 0

        # Generate the trajectory
        trajectory = []

        max_current_increment = (
            -self.max_current_increment[joint_index]
            if opposite_direction
            else self.max_current_increment[joint_index]
        )
        speed = (
            -self.speed[joint_index] if opposite_direction else self.speed[joint_index]
        )
        t_start = 0.0
        t_end = max_current_increment / speed
        t = t_start
        motor_current = initial_current

        while t <= t_end:
            trajectory.append(motor_current)
            t = t + dt
            motor_current = motor_current + dt * speed

        return trajectory

    def create_starting_points(
        self,
        number_of_starting_points: int,
        number_of_joints: int,
        lower_limits: npt.NDArray[np.float_],
        upper_limits: npt.NDArray[np.float_],
        safety_threshold: Optional[float] = 0.0,
    ) -> npt.NDArray[np.float_]:

        if (
            number_of_joints != lower_limits.size
            or number_of_joints != upper_limits.size
        ):
            raise ValueError(
                "{} The number of joints must be equal to the size of the lower and upper limits".format(
                    logPrefix
                )
            )

        starting_points = np.zeros((number_of_starting_points, number_of_joints))
        for joint_index in range(number_of_joints):
            tmp = np.linspace(
                lower_limits[joint_index] + safety_threshold,
                upper_limits[joint_index] - safety_threshold,
                number_of_starting_points + 2,
            )
            starting_points[:, joint_index] = tmp[1:-1]

        # repeat the starting points
        # each ramp has to be performed in both direction and for each speed
        speed_in = np.array(self.speed_in)
        speed_end = np.array(self.speed_end)
        speed_increment = np.array(self.speed_increment)
        repeats = 2 * int(
            np.floor(np.min((speed_end - speed_in) / speed_increment)) + 1
        )

        return np.repeat(starting_points, repeats=repeats, axis=0)


def build_remote_control_board_driver(
    param_handler: ParamHandler, local_prefix: str
) -> PolyDriver:
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_remote_control_board_remapper(param_handler)


def create_ctrl_c_handler(
    sensor_bridge: SensorBridge, robot_control: RobotControl
) -> Callable[[int, object], None]:
    def ctrl_c_handler(sig, frame):
        blf.log().info("{} Ctrl+C pressed. Exiting gracefully.".format(logPrefix))
        # get the feedback
        if not sensor_bridge.advance():
            raise RuntimeError(
                "{} Unable to advance the sensor bridge".format(logPrefix)
            )

        are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()

        if not are_joints_ok:
            raise RuntimeError("{} Unable to get the joint positions".format(logPrefix))

        # set the control mode to position
        if not robot_control.set_control_mode(
            blf.robot_interface.YarpRobotControl.Position
        ):
            raise RuntimeError("{} Unable to set the control mode".format(logPrefix))
        if not robot_control.set_references(
            joint_positions, blf.robot_interface.YarpRobotControl.Position
        ):
            raise RuntimeError("{} Unable to set the references".format(logPrefix))

        blf.log().info(
            "{} Sleep for two seconds. Just to be sure the interfaces are on.".format(
                logPrefix
            )
        )
        blf.clock().sleep_for(datetime.timedelta(seconds=2))
        sys.exit(0)

    return ctrl_c_handler


def main():

    # Create the argument parser
    arg_parser = argparse.ArgumentParser(
        description="This application drives the motors in current based on the trajectory type specified from the user."
    )

    # Add an argument for the trajectory type with default value "sinusoid"
    arg_parser.add_argument(
        "-t",
        "--trajectory",
        type=str,
        default="sinusoid",
        choices=["ramp", "sinusoid"],
        help="which trajectory (default: sinusoid)",
    )

    # Assign the content of the input argument to a variable
    trajectory_type = arg_parser.parse_args().trajectory
    blf.log().info("{} Trajectory type: {}".format(logPrefix, trajectory_type))

    # Check if running with Gazebo
    isGazebo = False
    if "gazebo" in (os.environ.get("YARP_ROBOT_NAME")).lower():
        isGazebo = True

    # Load parameters file
    param_handler = blf.parameters_handler.YarpParametersHandler()

    param_file = "blf-motor-current-tracking-options.ini"

    if not param_handler.set_from_filename(param_file):
        raise RuntimeError("{} Unable to load the parameters".format(logPrefix))

    # Load the trajectory parameters and create the trajectory generator
    if trajectory_type == "sinusoid":
        trajectory_generator = SinusoidTrajectoryGenerator.from_parameter_handler(
            param_handler.get_group("SINUSOID")
        )
    elif trajectory_type == "ramp":
        trajectory_generator = RampTrajectoryGenerator.from_parameter_handler(
            param_handler.get_group("RAMP")
        )

    # Load the motor parameters
    MotorParameters.from_parameter_handler(param_handler.get_group("MOTOR"))

    # Load joints to control and build the control board driver
    robot_control_handler = param_handler.get_group("ROBOT_CONTROL")
    joints_to_control = robot_control_handler.get_parameter_vector_string("joints_list")
    blf.log().info("{} Joints to control: {}".format(logPrefix, joints_to_control))
    # check if the joints are in the list of available joints
    for joint in joints_to_control:
        if joint not in MotorParameters.max_safety_current.keys():
            raise RuntimeError(
                "{} The joint {} is not supported by the application".format(
                    logPrefix, joint
                )
            )
    # check for which joints the motor current measure should be bypassed, when generating the reference trajectory
    bypass_motor_current_measure = param_handler.get_parameter_vector_bool(
        "bypass_motor_current_measure"
    )
    if len(bypass_motor_current_measure) != len(joints_to_control):
        raise ValueError(
            "{} The number of joints must be equal to the size of the bypass_motor_current_measure parameter".format(
                logPrefix
            )
        )

    trajectory_generator.set_joint_list(joints_to_control)

    poly_drivers = dict()

    poly_drivers["REMOTE_CONTROL_BOARD"] = build_remote_control_board_driver(
        param_handler=robot_control_handler,
        local_prefix="motor_current_tracking",
    )
    if not poly_drivers["REMOTE_CONTROL_BOARD"].is_valid():
        raise RuntimeError(
            "{} Unable to create the remote control board driver".format(logPrefix)
        )

    blf.log().info(
        "{} Sleep for two seconds. Just to be sure the interfaces are on.".format(
            logPrefix
        )
    )
    blf.clock().sleep_for(datetime.timedelta(seconds=2))

    robot_control = blf.robot_interface.YarpRobotControl()
    if not robot_control.initialize(robot_control_handler):
        raise RuntimeError(
            "{} Unable to initialize the robot control".format(logPrefix)
        )
    if not robot_control.set_driver(poly_drivers["REMOTE_CONTROL_BOARD"].poly):
        raise RuntimeError(
            "{} Unable to set the driver for the robot control".format(logPrefix)
        )

    # Get joint limits
    safety_threshold = param_handler.get_parameter_float("safety_threshold")
    safety_threshold = np.deg2rad(safety_threshold)
    use_safety_threshold_for_starting_position = param_handler.get_parameter_bool(
        "use_safety_threshold_for_starting_position"
    )
    _, lower_limits, upper_limits = robot_control.get_joint_limits()
    lower_limits = np.deg2rad(lower_limits)
    upper_limits = np.deg2rad(upper_limits)

    # Check whether softwares joint limits are available
    # If available, use them to reduce the range of motion
    try:
        software_lower_limits = param_handler.get_parameter_vector_float(
            "software_joint_lower_limits"
        )
        software_upper_limits = param_handler.get_parameter_vector_float(
            "software_joint_upper_limits"
        )
    except ValueError:
        software_lower_limits = None
        software_upper_limits = None

    if software_lower_limits is not None and software_upper_limits is not None:
        if len(software_lower_limits) != len(joints_to_control) or len(software_upper_limits) != len(joints_to_control):
            raise ValueError(
                "{} The number of joints must be equal to the size of the software joint limits".format(
                    logPrefix
                )
            )
        lower_limits = np.maximum(lower_limits, np.deg2rad(software_lower_limits))
        upper_limits = np.minimum(upper_limits, np.deg2rad(software_upper_limits))

    # Create the sensor bridge
    sensor_bridge = blf.robot_interface.YarpSensorBridge()
    sensor_bridge_handler = param_handler.get_group("SENSOR_BRIDGE")
    if not sensor_bridge.initialize(sensor_bridge_handler):
        raise RuntimeError(
            "{} Unable to initialize the sensor bridge".format(logPrefix)
        )
    if not sensor_bridge.set_drivers_list(list(poly_drivers.values())):
        raise RuntimeError(
            "{} Unable to set the drivers for the sensor bridge".format(logPrefix)
        )
    if not sensor_bridge.advance():
        raise RuntimeError("{} Unable to advance the sensor bridge".format(logPrefix))

    are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
    if not are_joints_ok:
        raise RuntimeError("{} Unable to get the joint positions".format(logPrefix))

    # Create the vectors collection server for logging
    vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer()
    if not vectors_collection_server.initialize(
        param_handler.get_group("DATA_LOGGING")
    ):
        raise RuntimeError(
            "{} Unable to initialize the vectors collection server".format(logPrefix)
        )

    vectors_collection_server.populate_metadata(
        "motors::desired::current",
        joints_to_control,
    )
    vectors_collection_server.populate_metadata(
        "motors::control_mode::current",
        joints_to_control,
    )
    vectors_collection_server.finalize_metadata()
    blf.clock().sleep_for(datetime.timedelta(milliseconds=200))

    # Create the ctrl+c handler
    ctrl_c_handler = create_ctrl_c_handler(
        sensor_bridge=sensor_bridge, robot_control=robot_control
    )
    signal.signal(signal.SIGINT, ctrl_c_handler)

    # Check if tqdm is installed
    try:
        from tqdm import tqdm

        is_tqdm_installed = True
    except ImportError:
        blf.log().warn(
            "{} tqdm is not installed, the progress bar will not be shown".format(
                logPrefix
            )
        )
        is_tqdm_installed = False
        last_printed_progress = -1
        # hijack tqdm with a dummy function
        tqdm = lambda x: x

    # Set the time step
    dt = param_handler.get_parameter_datetime("dt")

    # Get the starting position: the trajectory will be generated for each starting position
    number_of_starting_points = param_handler.get_parameter_int(
        "number_of_starting_points"
    )
    starting_positions = trajectory_generator.create_starting_points(
        number_of_starting_points=number_of_starting_points,
        number_of_joints=len(joints_to_control),
        safety_threshold=safety_threshold
        if use_safety_threshold_for_starting_position
        else 0.0,
        lower_limits=lower_limits,
        upper_limits=upper_limits,
    )
    blf.log().info(
        "{} Starting positions: \n {}".format(logPrefix, np.rad2deg(starting_positions))
    )

    # Start the data collection
    blf.log().info(
        "{} Waiting for your input, press ENTER to start the data collection".format(
            logPrefix
        )
    )
    input()
    blf.log().info("{} Start".format(logPrefix))

    opposite_direction = False

    for counter, starting_position in enumerate(starting_positions):

        # drive the joints to the starting position
        if not robot_control.set_control_mode(
            blf.robot_interface.YarpRobotControl.Position
        ):
            raise RuntimeError("{} Unable to set the control mode".format(logPrefix))
        if not robot_control.set_references(
            starting_position, blf.robot_interface.YarpRobotControl.Position
        ):
            raise RuntimeError("{} Unable to set the references".format(logPrefix))

        # wait for the joints to reach the starting position
        is_motion_done = False
        while not is_motion_done:
            is_ok, is_motion_done, _, _ = robot_control.check_motion_done()
            if not is_ok:
                raise RuntimeError(
                    "{} Unable to check if the motion is done".format(logPrefix)
                )
            blf.clock().sleep_for(datetime.timedelta(milliseconds=200))
        blf.clock().sleep_for(datetime.timedelta(seconds=1))
        blf.log().info(
            "{} Joints moved to the starting position #{}, starting the trajectory".format(
                logPrefix, counter + 1
            )
        )

        # Generate the trajectory
        if not sensor_bridge.advance():
            raise RuntimeError(
                "{} Unable to advance the sensor bridge".format(logPrefix)
            )

        if isGazebo:
            are_joints_ok, joint_torques, _ = sensor_bridge.get_joint_torques()
            motor_currents = joint_torques * [
                MotorParameters.k_tau[joint] for joint in joints_to_control
            ]
            if not are_joints_ok:
                raise RuntimeError(
                    "{} Unable to get the joint torques".format(logPrefix)
                )
        else:
            are_joints_ok, motor_currents, _ = sensor_bridge.get_motor_currents()
            if not are_joints_ok:
                raise RuntimeError(
                    "{} Unable to get the motor current".format(logPrefix)
                )

        trajectories = []
        opposite_direction = not opposite_direction

        for joint_index in range(len(joints_to_control)):

            if trajectory_type == "ramp":

                if counter % (2 * number_of_starting_points) == 0:
                    trajectory_generator.reset_velocity(joint_index=joint_index)

                trajectories.append(
                    trajectory_generator.generate(
                        dt=dt.total_seconds(),
                        initial_current=motor_currents[joint_index]
                        if not (bypass_motor_current_measure[joint_index])
                        else 0,
                        joint_index=joint_index,
                        opposite_direction=opposite_direction,
                    )
                )

                if not (counter % 2 == 0):
                    trajectory_generator.increment_velocity(joint_index=joint_index)

            else:
                trajectories.append(
                    trajectory_generator.generate(
                        dt=dt.total_seconds(),
                        initial_current=motor_currents[joint_index]
                        if not (bypass_motor_current_measure[joint_index])
                        else 0,
                        joint_index=joint_index,
                        opposite_direction=False,
                    )
                )

        # reset control modes for current/torque
        control_modes = [
            blf.robot_interface.YarpRobotControl.Torque
            if isGazebo
            else blf.robot_interface.YarpRobotControl.Current
            for _ in joints_to_control
        ]

        # update motor control modes
        if not robot_control.set_control_mode(control_modes):
            raise RuntimeError("{} Unable to set the control mode".format(logPrefix))

        # reset variables
        is_out_of_safety_limits = [False for _ in joints_to_control]
        position_reference = starting_position
        traj_index = 0
        delta_traj_index = 1

        # loop over the trajectory
        trajectories_length = np.array([len(trajectory) for trajectory in trajectories])
        for _ in tqdm(range(trajectories_length.max())):
            tic = blf.clock().now()

            # get the feedback
            if not sensor_bridge.advance():
                raise RuntimeError(
                    "{} Unable to advance the sensor bridge".format(logPrefix)
                )

            are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
            if not are_joints_ok:
                raise RuntimeError(
                    "{} Unable to get the joint positions".format(logPrefix)
                )

            # check if the joints are within the safety limits
            # if not, stop the trajectory and set the related control mode to position
            for joint_index, joint in enumerate(joints_to_control):
                if is_out_of_safety_limits[joint_index]:
                    # nothing to do, it was already notified before
                    continue
                if (
                    np.abs(joint_positions[joint_index] - lower_limits[joint_index])
                    < safety_threshold
                ) or (
                    np.abs(joint_positions[joint_index] - upper_limits[joint_index])
                    < safety_threshold
                ):
                    # set the control mode to position
                    control_modes[
                        joint_index
                    ] = blf.robot_interface.YarpRobotControl.Position

                    # set the reference to the current position
                    position_reference[joint_index] = joint_positions[joint_index]

                    # update flag
                    is_out_of_safety_limits[joint_index] = True

                    # set control modes
                    if not robot_control.set_control_mode(control_modes):
                        raise RuntimeError(
                            "{} Unable to set the control mode".format(logPrefix)
                        )

                    blf.log().warn(
                        "{} Joint {} is out of the safety limits, stopping its trajectory and switching to Position control with reference position {}".format(
                            logPrefix, joint, position_reference[joint_index]
                        )
                    )

            # get the current/torque references
            current_reference = []
            for joint_idx, trajectory in enumerate(trajectories):
                # check if the trajectory is over
                if traj_index < len(trajectory):
                    if is_out_of_safety_limits[joint_idx]:
                        current_reference.append(0)
                    else:
                        if isGazebo:
                            current_reference.append(
                                trajectory[traj_index]
                                / MotorParameters.k_tau[joints_to_control[joint_idx]]
                            )
                        else:
                            # check if the current is within the safety limits
                            if (
                                np.abs(trajectory[traj_index])
                                > MotorParameters.max_safety_current[
                                    joints_to_control[joint_idx]
                                ]
                            ):
                                RuntimeError(
                                    "{} The current reference for joint {} is exceeding the safety limits, exiting the application".format(
                                        logPrefix, joints_to_control[joint_idx]
                                    )
                                )
                            current_reference.append(trajectory[traj_index])
                else:
                    # if the trajectory is over, switch to position control with
                    # the the posiition reference as the last measured position
                    current_reference.append(0)
                    position_reference[joint_idx] = joint_positions[joint_idx]
                    is_out_of_safety_limits[joint_idx] = True

            # merge the position and current/torque references depending on the control mode
            reference = np.where(
                is_out_of_safety_limits, position_reference, np.array(current_reference)
            )

            # send the references motor current (or joint torque for Gazebo)
            if not robot_control.set_references(
                reference, control_modes, joint_positions
            ):
                raise RuntimeError("{} Unable to set the references".format(logPrefix))

            # check if it is time to move to next starting position
            if all(is_out_of_safety_limits):
                blf.log().info(
                    "{} The trajectory is stopped due to all joints exceeding safety limits. Moving to next starting position, if available.".format(
                        logPrefix
                    )
                )
                break

            # log the data
            vectors_collection_server.prepare_data()
            if not vectors_collection_server.clear_data():
                raise RuntimeError("{} Unable to clear the data".format(logPrefix))
            vectors_collection_server.populate_data(
                "motors::desired::current", current_reference
            )
            vectors_collection_server.populate_data(
                "motors::control_mode::current", np.where(is_out_of_safety_limits, 0, 1)
            )
            vectors_collection_server.send_data()

            # sleep
            toc = blf.clock().now()
            delta_time = toc - tic
            if delta_time < dt:
                blf.clock().sleep_for(dt - delta_time)
            else:
                blf.log().debug(
                    "{} The control loop is too slow, real time constraints not satisfied".format(
                        logPrefix
                    )
                )

            # Print progress percentage, only if tqdm is not installed
            if not is_tqdm_installed:

                # Calculate progress
                progress = (traj_index + 1) / trajectories_length.max() * 100

                # Check if progress is a multiple of 10 and different from last printed progress
                if int(progress) % 10 == 0 and int(progress) != last_printed_progress:
                    print(f"Progress: {int(progress)}%", end="\r")
                    last_printed_progress = int(progress)

            # update the trajectory index
            traj_index = traj_index + delta_traj_index

    blf.log().info("{} Data colection completed".format(logPrefix))

    # get the feedback
    if not sensor_bridge.advance():
        raise RuntimeError("{} Unable to advance the sensor bridge".format(logPrefix))

    are_joints_ok, joint_positions, _ = sensor_bridge.get_joint_positions()

    if not are_joints_ok:
        raise RuntimeError("{} Unable to get the joint positions".format(logPrefix))

    # set the control mode to position
    if not robot_control.set_control_mode(
        blf.robot_interface.YarpRobotControl.Position
    ):
        raise RuntimeError("{} Unable to set the control mode".format(logPrefix))
    if not robot_control.set_references(
        joint_positions, blf.robot_interface.YarpRobotControl.Position
    ):
        raise RuntimeError("{} Unable to set the references".format(logPrefix))


if __name__ == "__main__":
    network = yarp.Network()
    main()
