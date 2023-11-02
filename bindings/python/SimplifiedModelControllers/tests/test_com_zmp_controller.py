import pytest
import bipedal_locomotion_framework.bindings as blf

pytestmark = pytest.mark.CoMZMPController


def test_com_zmp_controller():
    controller = blf.simplified_model_controllers.CoMZMPController()

    # Defining the param handler
    controller_param_handler = blf.parameters_handler.StdParametersHandler()
    controller_param_handler.set_parameter_vector_float("zmp_gain", [3.0, 4.0])
    controller_param_handler.set_parameter_vector_float("com_gain", [1.0, 2.0])

    # define the input
    controller_input = blf.simplified_model_controllers.CoMZMPControllerInput()
    controller_input.desired_CoM_position = [0.01, 0]
    controller_input.desired_CoM_velocity = [0.0, 0]
    controller_input.desired_ZMP_position = [0.01, 0]
    controller_input.CoM_position = [0.01, 0]
    controller_input.ZMP_position = [0.01, 0]
    controller_input.angle = 0
    assert controller.initialize(controller_param_handler)
    assert controller.set_input(controller_input)
    assert controller.advance()
    assert controller.is_output_valid()
