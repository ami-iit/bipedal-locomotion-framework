import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import numpy as np


def test_unicycle_knot():

    knot1 = blf.planners.UnicycleKnot()

    assert knot1.x == 0.0
    assert knot1.y == 0.0
    assert knot1.dx == 0.0
    assert knot1.dy == 0.0
    assert knot1.time == 0.0

    knot2 = blf.planners.UnicycleKnot(position=(1.0, 2.0),
                                      velocity=(0.1, 0.2),
                                      time=0.5)

    assert knot2.x == 1.0
    assert knot2.y == 2.0
    assert knot2.dx == 0.1
    assert knot2.dy == 0.2
    assert knot2.time == 0.5


def test_unicycle_planner_input():

    unicycle_input1 = blf.planners.UnicyclePlannerInput()

    assert unicycle_input1.t0 == 0.0
    assert unicycle_input1.tf == 0.0
    assert unicycle_input1.knots == []

    knot1 = blf.planners.UnicycleKnot(
        position=(1.0, 0.0), velocity=(0.1, 0.2), time=0.5)
    knot2 = blf.planners.UnicycleKnot(
        position=(2.0, 1.0), velocity=(0.0, -0.2), time=1.5)
    unicycle_input2 = blf.planners.UnicyclePlannerInput(
        t0=1.0, tf=2.0, knots=[knot1, knot2])

    assert unicycle_input2.t0 == 1.0
    assert unicycle_input2.tf == 2.0
    assert unicycle_input2.knots == [knot1, knot2]


def test_unicycle_planner_output():

    unicycle_output1 = blf.planners.UnicyclePlannerOutput()

    assert unicycle_output1.left == blf.contacts.ContactList()
    assert unicycle_output1.right == blf.contacts.ContactList()

    contact_left1 = blf.contacts.PlannedContact()
    contact_left1.name = "ContactLeft1"
    contact_left1.activation_time = 0.1
    contact_left1.deactivation_time = 0.5

    contact_right1 = blf.contacts.PlannedContact()
    contact_right1.name = "ContactRight1"
    contact_right1.activation_time = 0.5
    contact_right1.deactivation_time = 2.5

    left = blf.contacts.ContactList()
    left.add_contact(contact_left1)

    right = blf.contacts.ContactList()
    right.add_contact(contact_right1)

    unicycle_output2 = blf.planners.UnicyclePlannerOutput(
        left=left, right=right)

    assert unicycle_output2.left == left
    assert unicycle_output2.right == right


def test_unicycle_planner(capsys):

    parameters_handler = blf.parameters_handler.StdParametersHandler()

    # dt
    parameters_handler.set_parameter_float("sampling_time", 0.010)

    # gains
    parameters_handler.set_parameter_float("unicycleGain", 10.0)
    parameters_handler.set_parameter_float("slowWhenTurningGain", 5.0)

    # reference
    parameters_handler.set_parameter_vector_float("referencePosition", (0.1, 0.0))

    # weights
    parameters_handler.set_parameter_float("timeWeight", 2.5)
    parameters_handler.set_parameter_float("positionWeight", 1.0)

    # duration
    parameters_handler.set_parameter_float("minStepDuration", 0.8)
    parameters_handler.set_parameter_float("maxStepDuration", 2.0)
    parameters_handler.set_parameter_float("nominalDuration", 0.9)

    # step length
    parameters_handler.set_parameter_float("minStepLength", 0.05)
    parameters_handler.set_parameter_float("maxStepLength", 0.30)

    # feet distance
    parameters_handler.set_parameter_float("minWidth", 0.14)
    parameters_handler.set_parameter_float("nominalWidth", 0.16)

    # angle variation
    parameters_handler.set_parameter_float("minAngleVariation", np.deg2rad(8.0))
    parameters_handler.set_parameter_float("maxAngleVariation", np.deg2rad(15.0))

    # gait
    parameters_handler.set_parameter_float("switchOverSwingRatio", 0.6)

    # Create the planner
    unicycle = blf.planners.UnicyclePlanner()

#    with capsys.disabled():
#        assert not unicycle.set_input(input=blf.planners.UnicyclePlannerInput())
#        assert not unicycle.is_output_valid()
#        assert not unicycle.advance()

    # Initialize the planner
    assert unicycle.initialize(handler=parameters_handler)

    # Create the input
    unicycle_input = blf.planners.UnicyclePlannerInput(
        t0=0.0, tf=25.0, knots=[
            blf.planners.UnicycleKnot(position=(0.0, 0.0), velocity=(0.0, 0.0), time=0.0),
            blf.planners.UnicycleKnot(position=(0.5, 0.0), velocity=(0.1, 0.0), time=10.0),
            blf.planners.UnicycleKnot(position=(1.0, 0.25), velocity=(0.1, 0.1), time=20.0),
            blf.planners.UnicycleKnot(position=(1.1, 0.25), velocity=(0.0, 0.0), time=25.0),
        ])

    # Set the input
    assert unicycle.set_input(input=unicycle_input)

    # Compute the steps
    assert unicycle.advance()

    # Get the output
    unicycle_output = unicycle.get_output()

    # Check that steps have been computed
    assert len(unicycle_output.left) > 0
    assert len(unicycle_output.right) > 0

    for contact in unicycle_output.left:
        print(contact)

    print()

    for contact in unicycle_output.right:
        print(contact)
