import pytest

pytestmark = pytest.mark.contact_detectors

import bipedal_locomotion_framework.bindings as blf
import manifpy as manif
import numpy as np


def test_fixed_foot_detector():
    dt = 0.01

    parameters_handler = blf.parameters_handler.StdParametersHandler()
    parameters_handler.set_parameter_float("sampling_time", dt)

    detector = blf.contacts.FixedFootDetector()
    assert (detector.initialize(parameters_handler))

    # Create the map of contact lists
    contact_list_map = dict()

    # Names of the frames
    rfoot_frame = "r_sole"
    lfoot_frame = "l_sole"

    # Create the contact lists
    contact_list_map[rfoot_frame] = blf.contacts.ContactList()
    contact_list_map[lfoot_frame] = blf.contacts.ContactList()

    # Set the contacts (t: time, L: left foot, R: right foot)

    # t:   ||    1    2    3    4    5    6    7    8    9    10
    #      ||----|----|----|----|----|----|----|----|----|----||
    # L:   ||****|****|****|    |****|****|****|****|****|****||
    # R:   ||****|****|****|****|****|****|    |****|****|****||

    # Initial foot position and orientation
    r_foot_quat = [1, 0, 0, 0]
    r_foot_position = [0, 0.7, 0]
    l_foot_quat = [1, 0, 0, 0]
    l_foot_position = [0, -0.7, 0]

    assert contact_list_map[lfoot_frame].add_contact(
        transform=manif.SE3(position=np.array(l_foot_position),
                            quaternion=l_foot_quat),
        activation_time=0.0,
        deactivation_time=3.0)
    assert contact_list_map[rfoot_frame].add_contact(
        transform=manif.SE3(position=np.array(r_foot_position),
                            quaternion=r_foot_quat),
        activation_time=0.0,
        deactivation_time=6.0)

    assert contact_list_map[lfoot_frame].add_contact(
        transform=manif.SE3(position=np.array([l_foot_position[0] + 0.1, l_foot_position[1], 0.0]),
                            quaternion=l_foot_quat),
        activation_time=4.0,
        deactivation_time=10.0)
    assert contact_list_map[rfoot_frame].add_contact(
        transform=manif.SE3(position=np.array([r_foot_position[0] + 0.2, r_foot_position[1], 0.0]),
                            quaternion=r_foot_quat),
        activation_time=7.0,
        deactivation_time=10.0)

    phase_list = blf.contacts.ContactPhaseList()
    phase_list.set_lists(contact_lists=contact_list_map)

    assert (detector.set_contact_phase_list(phase_list))

    # Retrieve initial fixed foot
    fixed_foot = detector.get_fixed_foot()

    assert (fixed_foot.name == "l_sole")
    assert (fixed_foot.pose == manif.SE3(position=np.array(l_foot_position), quaternion=l_foot_quat))
    assert (fixed_foot.switch_time == 0.0)
    assert (fixed_foot.last_update_time == 0.0)

    # Retrieve fixed foot along the trajectory
    for i in np.arange(start=dt, step=dt, stop=10.0):

        assert (detector.advance())
        fixed_foot = detector.get_fixed_foot()

        # Fixed foot: left - Phase: double support
        if i <= 3.0:
            assert (fixed_foot.name == "l_sole")
            assert (fixed_foot.pose == manif.SE3(position=np.array(l_foot_position), quaternion=l_foot_quat))
            assert (fixed_foot.switch_time == 0.0)
            assert (round(fixed_foot.last_update_time, 2) == round(i, 2)) # last_update_time updated incrementally

        # Fixed foot: right - Phase: single support
        elif i <= 4.0:
            assert (fixed_foot.name == "r_sole")
            assert (fixed_foot.pose == manif.SE3(position=np.array(r_foot_position), quaternion=r_foot_quat))
            assert (fixed_foot.switch_time == 3.0)
            assert (round(fixed_foot.last_update_time, 2) == round(i, 2)) # last_update_time updated incrementally

        # Fixed foot: right - Phase: double support
        elif i <= 6.0:
            assert (fixed_foot.name == "r_sole")
            assert (fixed_foot.pose == manif.SE3(position=np.array(r_foot_position), quaternion=r_foot_quat))
            assert (fixed_foot.switch_time == 3.0)
            assert (round(fixed_foot.last_update_time, 2) == 4.0) # last_update_time fixed to the beginning of DS

        # Fixed foot: left - Phase: single support
        elif i <= 7.0:
            assert (fixed_foot.name == "l_sole")
            assert (fixed_foot.pose == manif.SE3(
                position=np.array(np.array([l_foot_position[0] + 0.1, l_foot_position[1], 0.0])),
                quaternion=l_foot_quat))
            assert (fixed_foot.switch_time == 6.0)
            assert (round(fixed_foot.last_update_time, 2) == round(i, 2)) # last_update_time updated incrementally

        # Fixed foot: left - Phase: double support
        elif i <= 10.0:
            assert (fixed_foot.name == "l_sole")
            assert (fixed_foot.pose == manif.SE3(
                position=np.array(np.array([l_foot_position[0] + 0.1, l_foot_position[1], 0.0])),
                quaternion=l_foot_quat))
            assert (fixed_foot.switch_time == 6.0)
            assert (round(fixed_foot.last_update_time, 2) == 7.0) # last_update_time fixed to the beginning of DS
