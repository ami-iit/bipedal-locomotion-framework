import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import manifpy as manif
import numpy as np


def test_swing_foot_planner_state():

    state = blf.planners.SwingFootPlannerState()

    assert state.is_in_contact == True
    state.is_in_contact = False
    assert state.is_in_contact == False

    state.transform = manif.SE3(position=np.array([0.0, -0.2, 0.5]),
                                quaternion=np.array([0, 0, 0, 1]))
    assert state.transform.translation() == pytest.approx([0.0, -0.2, 0.5])
    assert state.transform.quat() == pytest.approx([0, 0, 0, 1])

    state.mixed_velocity = np.array([0.1, 0.2, 0.3, -0.1, -0.2, -0.3])
    assert state.mixed_velocity == pytest.approx([0.1, 0.2, 0.3, -0.1, -0.2, -0.3])

    state.mixed_acceleration = np.array([0, 0.02, 0.03, -0.01, -0.02, -0.03])
    assert state.mixed_acceleration == pytest.approx([0, 0.02, 0.03, -0.01, -0.02, -0.03])


def test_swing_foot_planner():

    contact_list = blf.contacts.ContactList()

    from scipy.spatial.transform import Rotation as R

    # First footstep
    tf1 = manif.SE3(position=[0.9, 0, 0],
                    quaternion=R.from_euler(seq="z", angles=np.pi / 2).as_quat())
    assert contact_list.add_contact(transform=tf1,
                                    activation_time=0.0,
                                    deactivation_time=0.3)

    # Second footstep
    tf2 = manif.SE3(position=[0.8899, 0.1345, 0.0600],
                    quaternion=R.from_euler(seq="z", angles=1.7208).as_quat())
    assert contact_list.add_contact(transform=tf2,
                                    activation_time=0.6,
                                    deactivation_time=1.5)

    # Third footstep
    tf3 = manif.SE3(position=[0.8104, 0.3915, 0.1800],
                    quaternion=R.from_euler(seq="z", angles=2.0208).as_quat())
    assert contact_list.add_contact(transform=tf3,
                                    activation_time=1.8,
                                    deactivation_time=2.7)

    # Fourth footstep
    tf4 = manif.SE3(position=[0.6585, 0.6135, 0.3000],
                    quaternion=R.from_euler(seq="z", angles=2.3208).as_quat())
    assert contact_list.add_contact(transform=tf4,
                                    activation_time=3.0,
                                    deactivation_time=3.9)

    # Fifth footstep
    tf5 = manif.SE3(position=[0.3261, 0.8388, 0.4800],
                    quaternion=R.from_euler(seq="z", angles=2.7708).as_quat())
    assert contact_list.add_contact(transform=tf5,
                                    activation_time=4.2,
                                    deactivation_time=6.0)

    dT = 0.01

    parameters_handler = blf.parameters_handler.StdParametersHandler()
    parameters_handler.set_parameter_float("sampling_time", dT)
    parameters_handler.set_parameter_float("step_height", 0.1)
    parameters_handler.set_parameter_float("foot_apex_time", 0.5)
    parameters_handler.set_parameter_float("foot_landing_velocity", 0.0)
    parameters_handler.set_parameter_float("foot_landing_acceleration", 0.0)
    parameters_handler.set_parameter_float("foot_take_off_velocity", 0.0)
    parameters_handler.set_parameter_float("foot_take_off_acceleration", 0.0)

    planner = blf.planners.SwingFootPlanner()
    assert planner.initialize(handler=parameters_handler)
    planner.set_contact_list(contact_list=contact_list)

    num_of_iterations = (contact_list[len(contact_list) - 1].deactivation_time + 1) / dT

    for i in range(int(num_of_iterations)):

        # state = planner.get()
        # print(state.transform[0:3], state.transform[3:])

        assert planner.advance()
