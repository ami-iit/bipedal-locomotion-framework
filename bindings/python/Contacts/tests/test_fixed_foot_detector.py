import pytest

pytestmark = pytest.mark.contact_detectors

import bipedal_locomotion_framework.bindings as blf
import manifpy as manif
import numpy as np

from datetime import timedelta
from typing import Dict

class FixedFootState:
    def __init__(self):
        self.left_foot = blf.contacts.EstimatedContact()
        self.right_foot = blf.contacts.EstimatedContact()

def get_fixed_foot_state(t: timedelta, list_map: Dict[str, blf.contacts.ContactList]) -> FixedFootState:

    # t            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    # L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    # R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    # stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|
    state = FixedFootState()
    state.left_foot.pose = list_map["left_foot"].get_present_contact(t).pose
    state.right_foot.pose = list_map["right_foot"].get_present_contact(t).pose

    if t < timedelta(seconds=1):
        state.left_foot.is_active = True
        state.right_foot.is_active = False
    elif t < timedelta(seconds=3):
        state.left_foot.is_active = False
        state.right_foot.is_active = True
    elif t < timedelta(seconds=5):
        state.left_foot.is_active = True
        state.right_foot.is_active = False
    elif t < timedelta(seconds=7):
        state.left_foot.is_active = False
        state.right_foot.is_active = True
    elif t < timedelta(seconds=9):
        state.left_foot.is_active = True
        state.right_foot.is_active = False
    elif t < timedelta(seconds=11):
        state.left_foot.is_active = False
        state.right_foot.is_active = True
    elif t < timedelta(seconds=13):
        state.left_foot.is_active = True
        state.right_foot.is_active = False
    elif t < timedelta(seconds=15):
        state.left_foot.is_active = False
        state.right_foot.is_active = True
    else:
        state.left_foot.is_active = True
        state.right_foot.is_active = False

    return state

def create_contact_list() -> blf.contacts.ContactPhaseList:
    # t            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    # L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    # R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    # stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|

    contact_list_map = dict()
    contact_list_map["left_foot"] = blf.contacts.ContactList()
    contact_list_map["right_foot"] = blf.contacts.ContactList()

    left_position = np.array([0.0, 0.08, 0.0])
    # quaternion=[0, 0, 0, 1] please read it as 0i + 0j + 0k + 1
    left_transform = manif.SE3(left_position, [0, 0, 0, 1])
    assert(contact_list_map["left_foot"].add_contact(left_transform,
                                                      timedelta(seconds=0),
                                                      timedelta(seconds=1)))

    left_position[0] += 0.05
    left_transform.translation(left_position)
    assert(contact_list_map["left_foot"].add_contact(left_transform,
                                                     timedelta(seconds=2),
                                                     timedelta(seconds=5)))

    left_position[0] += 0.1
    left_transform.translation(left_position)
    assert(contact_list_map["left_foot"].add_contact(left_transform,
                                                     timedelta(seconds=6),
                                                     timedelta(seconds=9)))

    left_position[0] += 0.1
    left_transform.translation(left_position)
    assert(contact_list_map["left_foot"].add_contact(left_transform,
                                                     timedelta(seconds=10),
                                                     timedelta(seconds=13)))

    left_position[0] += 0.1
    left_transform.translation(left_position)
    assert(contact_list_map["left_foot"].add_contact(left_transform,
                                                     timedelta(seconds=14),
                                                     timedelta(seconds=17)))


    right_position = np.array([0.0, -0.08, 0.0])
    # quaternion=[0, 0, 0, 1] please read it as 0i + 0j + 0k + 1
    right_transform = manif.SE3(right_position, [0, 0, 0, 1])

    assert(contact_list_map["right_foot"].add_contact(right_transform,
                                                      timedelta(seconds=0),
                                                      timedelta(seconds=3)))

    right_position[0] += 0.1
    right_transform.translation(right_position)
    assert(contact_list_map["right_foot"].add_contact(right_transform,
                                                      timedelta(seconds=4),
                                                      timedelta(seconds=7)))

    right_position[0] += 0.1
    right_transform.translation(right_position)
    assert(contact_list_map["right_foot"].add_contact(right_transform,
                                                      timedelta(seconds=8),
                                                      timedelta(seconds=11)))

    right_position[0] += 0.1
    right_transform.translation(right_position)
    assert(contact_list_map["right_foot"].add_contact(right_transform,
                                                      timedelta(seconds=12),
                                                      timedelta(seconds=15)))

    right_position[0] += 0.05
    right_transform.translation(right_position)
    assert(contact_list_map["right_foot"].add_contact(right_transform,
                                                      timedelta(seconds=16),
                                                      timedelta(seconds=17)))


    phase_list = blf.contacts.ContactPhaseList()
    phase_list.set_lists(contact_list_map)

    return phase_list


def time_generator(start: timedelta, stop: timedelta, step: timedelta):
  while start < stop:
    yield start
    start = start + step

def test_fixed_foot_detector():
    dt = timedelta(milliseconds=10)
    horizon = timedelta(seconds=20)

    detector = blf.contacts.FixedFootDetector()
    parameters_handler = blf.parameters_handler.StdParametersHandler()
    parameters_handler.set_parameter_datetime("sampling_time", dt)

    assert (detector.initialize(parameters_handler))

    phase_list = create_contact_list()
    detector.set_contact_phase_list(phase_list)

    for current_time in time_generator(timedelta(seconds=0), horizon, dt):

        assert(detector.advance())
        assert(detector.is_output_valid())
        output = detector.get_output()

        state = get_fixed_foot_state(current_time, phase_list.lists())
        assert(output["left_foot"].is_active == state.left_foot.is_active)
        assert(output["right_foot"].is_active == state.right_foot.is_active)

        if state.left_foot.is_active:
            assert(output["left_foot"].pose == state.left_foot.pose)
        elif state.right_foot.is_active:
            assert(output["right_foot"].pose == state.right_foot.pose)
        else:
            assert(False) # This should never happen
