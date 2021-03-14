import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import numpy as np


def test_contact():

    contact = blf.contacts.PlannedContact()

    # Default values
    assert contact.pose == blf.manif.SE3(position=[0., 0, 0], quaternion=[0, 0, 0, 1])
    assert contact.activation_time == 0.0
    assert contact.deactivation_time == 0.0
    assert contact.name == "Contact"
    assert contact.type == blf.contacts.ContactType.Full

    contact.pose = blf.manif.SE3(position=[1.0, -2.0, 3.3], quaternion=[0, 0, -1, 0])
    assert contact.pose.position == pytest.approx(np.array([1.0, -2.0, 3.3]))
    assert contact.pose.quaternion == pytest.approx(np.array([0, 0, -1, 0]))

    contact.activation_time = 42.0
    assert contact.activation_time == 42.0

    contact.deactivation_time = -42.0
    assert contact.deactivation_time == -42.0

    contact.name = "MyNewContact"
    assert contact.name == "MyNewContact"

    contact.type = blf.contacts.ContactType.Point
    assert contact.type == blf.contacts.ContactType.Point


def test_contact_list():

    contact_list = blf.contacts.ContactList()

    assert len(contact_list) == contact_list.size() == 0
    assert contact_list.default_name() == "ContactList"
    assert contact_list.default_contact_type() == blf.contacts.ContactType.Full

    contact_list.set_default_name(default_name="MyContactList")
    assert contact_list.default_name() == "MyContactList"

    contact_list.set_default_contact_type(type=blf.contacts.ContactType.Point)
    assert contact_list.default_contact_type() == blf.contacts.ContactType.Point

    contact1 = blf.contacts.PlannedContact()
    contact1.name = "Contact1"
    contact1.activation_time = 0.1
    contact1.deactivation_time = 0.5

    contact2 = blf.contacts.PlannedContact()
    contact2.name = "Contact2"
    contact2.activation_time = 1.0
    contact2.deactivation_time = 1.5

    assert contact_list.add_contact(contact1)
    assert contact_list.add_contact(contact2)

    assert contact_list[0] == contact1
    assert contact_list[len(contact_list) - 1] == contact2  # TODO: improve

    contact3 = blf.contacts.PlannedContact()
    contact3.name = "Contact3"
    contact3.activation_time = 0.6
    contact3.deactivation_time = 0.8

    assert contact_list.add_contact(contact3)
    assert len(contact_list) == 3
    assert contact_list[1] == contact3

    contact3_bis = blf.contacts.PlannedContact()
    contact3_bis.name = "Contact3"
    contact3_bis.activation_time = 0.9
    contact3_bis.deactivation_time = 1.6

    assert not contact_list.add_contact(contact3_bis)

    contact2_modified = blf.contacts.PlannedContact()
    contact2_modified.name = "Contact2Modified"
    contact2_modified.type = blf.contacts.ContactType.Point

    # TODO
    # contact_list[len(contact_list) - 1] = contact2_modified
    # assert contact_list[len(contact_list) - 1] == contact2_modified

    assert contact2 == contact_list.get_present_contact(time=1.2)
    assert contact2 == contact_list.get_present_contact(time=1.6)
    assert contact3 == contact_list.get_present_contact(time=0.6)
    # assert contact_list.get_present_contact(time=0.0) == \
    #        contact_list[len(contact_list) - 1]

    contact_list.clear()
    assert len(contact_list) == contact_list.size() == 0

    for i in range(50):

        assert contact_list.add_contact(
            transform=blf.manif.SE3(position=np.array([0, 0, 0]),
                                    quaternion=np.array([0, 0, 0, 1.])),
            activation_time=2.0 + i,
            deactivation_time=2.5 + i)

    assert len(contact_list) == contact_list.size() == 50

    for idx, contact in enumerate(contact_list):

        assert contact == contact_list[idx]


def test_contact_phase():

    phase = blf.contacts.ContactPhase()

    # Default values
    assert phase.begin_time == 0.0
    assert phase.end_time == 0.0
    assert phase.active_contacts == dict()

    list1 = blf.contacts.ContactList()
    list1.set_default_name(default_name="List1")

    list2 = blf.contacts.ContactList()
    list2.set_default_name(default_name="List2")

    # TODO: the active_contacts is a read_only attribute
