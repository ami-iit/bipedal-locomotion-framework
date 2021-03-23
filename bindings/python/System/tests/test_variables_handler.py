import pytest
pytestmark = pytest.mark.system

import bipedal_locomotion_framework.bindings.system as blf

def test_variables_handler():

    variable1_size = 42;
    variable2_size = 35;

    handler = blf.VariablesHandler()
    assert handler.add_variable("variable_1", variable1_size) is True
    assert handler.add_variable("variable_2", variable2_size) is True

    assert handler.get_variable("variable_1").offset == 0
    assert handler.get_variable("variable_1").size == variable1_size

    assert handler.get_variable("variable_2").offset == variable1_size
    assert handler.get_variable("variable_2").size == variable2_size

    assert handler.get_number_of_variables() == variable1_size + variable2_size

    assert handler.get_variable("variable_3").is_valid() is False
