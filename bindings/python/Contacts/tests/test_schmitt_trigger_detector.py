import pytest
pytestmark = pytest.mark.contact_detectors

import bipedal_locomotion_framework.bindings as blf


def test_schmitt_trigger_detector():

    parameters_handler = blf.parameters_handler.StdParametersHandler()
    parameters_handler.set_parameter_vector_string("contacts", ["right"])
    parameters_handler.set_parameter_vector_float("contact_make_thresholds", [100.0])
    parameters_handler.set_parameter_vector_float("contact_break_thresholds", [10.0])
    parameters_handler.set_parameter_vector_float("contact_make_switch_times", [0.2])
    parameters_handler.set_parameter_vector_float("contact_break_switch_times", [0.2, 0.2])

    detector = blf.contacts.SchmittTriggerDetector()
    assert(detector.initialize(parameters_handler) == False)

    parameters_handler.set_parameter_vector_float("contact_break_switch_times", [0.2])
    assert(detector.initialize(parameters_handler))

    assert(detector.reset_contacts())

    # rise signal
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.1, raw_value=120.0)))
    assert(detector.advance())
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.2, raw_value=120.0)))
    assert(detector.advance())
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.3, raw_value=120.0)))
    assert(detector.advance())

    # contact state should turn true
    right_contact = detector.get("right")
    assert(right_contact.is_active)
    assert(right_contact.switch_time == 0.3)

    # fall signal
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.4, raw_value=7.0)))
    assert(detector.advance())
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.5, raw_value=7.0)))
    assert(detector.advance())
    assert(detector.set_timed_trigger_input("right",
                                            blf.math.SchmittTriggerInput(time=0.6, raw_value=7.0)))
    assert(detector.advance())

    # contact state should turn false
    right_contact = detector.get("right")
    assert(right_contact.is_active == False)
    assert(right_contact.switch_time == 0.6)

    # add a new contact
    params = blf.math.SchmittTrigger.Params(off_threshold=10, on_threshold=100,
                                            switch_off_after=0.2, switch_on_after=0.2)
    detector.add_contact("left",
                         blf.math.SchmittTriggerState(state=False, switch_time=0.6, edge_time=0.6),
                         params)
    contacts = detector.get_output()
    assert(len(contacts) == 2)
    assert(contacts["right"].is_active == False)

    # test multiple measurement updates
    right_input = blf.math.SchmittTriggerInput()
    right_input.time = 0.7
    right_input.raw_value = 120
    left_input = blf.math.SchmittTriggerInput()
    left_input.time = 0.7
    left_input.raw_value = 120
    timed_inputs = {"right":right_input, "left":left_input}
    assert(detector.set_timed_trigger_inputs(timed_inputs))


    # test removing a contact
    assert(detector.remove_contact("left"))
    contacts = detector.get_output()
    assert(len(contacts) == 1)

    # test resetting a contact
    assert(detector.reset_contact("right", True, params))
    right_contact = detector.get("right")
    assert(right_contact.is_active == True)
