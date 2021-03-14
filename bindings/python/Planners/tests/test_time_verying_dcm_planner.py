import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import numpy as np


def test_dcm_planner_state():

    state = blf.planners.DCMPlannerState()

    state.dcm_position = [1., -2, 3.14]
    assert state.dcm_position == pytest.approx([1., -2, 3.14])

    state.dcm_velocity = [0.1, -0.2, 0.314]
    assert state.dcm_velocity == pytest.approx([0.1, -0.2, 0.314])

    state.vrp_position = [-0.5, 0.0, 42.42]
    assert state.vrp_position == pytest.approx([-0.5, 0.0, 42.42])

    state.omega = 42.0
    assert state.omega == pytest.approx(42.0)

    state.omega_dot = 2.7182
    assert state.omega_dot == pytest.approx(2.7182)


def test_time_varying_dcm_planner():

    # Create the map of contact lists
    contact_list_map = dict()

    # Left foot
    contact_list_map["left"] = blf.contacts.ContactList()

    # L1: first footstep
    assert contact_list_map["left"].add_contact(
        transform=blf.manif.SE3(position=np.array([0, -0.8, 0]),
                          quaternion=np.array([0, 0, 0, 1])),
        activation_time=0.0,
        deactivation_time=1.0)

    # L2: second footstep
    assert contact_list_map["left"].add_contact(
        transform=blf.manif.SE3(position=np.array([0.25, -0.8, 0.2]),
                                quaternion=np.array([0, 0, 0, 1])),
        activation_time=2.0,
        deactivation_time=7.0)

    # Right foot
    contact_list_map["right"] = blf.contacts.ContactList()

    # R1: first footstep
    assert contact_list_map["right"].add_contact(
        transform=blf.manif.SE3(position=np.array([0, 0.8, 0]),
                                quaternion=np.array([0, 0, 0, 1])),
        activation_time=0.0,
        deactivation_time=3.0)

    # R2: second footstep
    assert contact_list_map["right"].add_contact(
        transform=blf.manif.SE3(position=np.array([0.25, 0.8, 0.2]),
                                quaternion=np.array([0, 0, 0, 1])),
        activation_time=4.0,
        deactivation_time=7.0)

    # Create the contact phase list
    phase_list = blf.contacts.ContactPhaseList()
    phase_list.set_lists(contact_list_map)

    # Set the parameters
    handler = blf.parameters_handler.StdParametersHandler()
    handler.set_parameter_float(name="planner_sampling_time", value=0.05)
    handler.set_parameter_int(name="number_of_foot_corners", value=4)

    # Set the foot corners
    handler.set_parameter_vector_float(name="foot_corner_0", value=[0.1, 0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_1", value=[0.1, -0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_2", value=[-0.1, -0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_3", value=[-0.1, 0.05, 0.0])

    # Set the weight of the cost function
    handler.set_parameter_float("omega_dot_weight", 1.0)
    handler.set_parameter_float("dcm_tracking_weight", 1.0)
    handler.set_parameter_float("omega_dot_rate_of_change_weight", 10.0)
    handler.set_parameter_float("vrp_rate_of_change_weight", 100.0)
    handler.set_parameter_float("dcm_rate_of_change_weight", 1.0)

    # Set the initial state
    initial_state = blf.planners.DCMPlannerState()
    initial_state.dcm_position = np.array([0, 0, 0.53])
    initial_state.dcm_velocity = np.zeros(3)
    initial_state.vrp_position = initial_state.dcm_position
    initial_state.omega = np.sqrt(blf.math.StandardAccelerationOfGravitation / initial_state.dcm_position[2])

    # Initialize the planner
    planner = blf.planners.TimeVaryingDCMPlanner()
    assert planner.initialize(handler=handler)

    assert planner.set_contact_phase_list(contact_phase_list=phase_list)
    planner.set_initial_state(state=initial_state)

    assert planner.compute_trajectory()

    for _ in range(150):

        assert planner.advance()
