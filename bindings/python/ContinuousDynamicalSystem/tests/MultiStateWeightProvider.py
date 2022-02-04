import pytest
pytestmark = pytest.mark.continuous_dynamical_system

import bipedal_locomotion_framework.bindings as blf
import numpy as np

def test_custom_task():
    dt = 0.01
    settling_time = 0.1
    tolerance = 1e-2

    walking_weight = np.array([10, 100, 1000])
    stance_weight = np.array([0, -10, 20])

    params = blf.parameters_handler.StdParametersHandler()
    params.set_parameter_float("settling_time", settling_time)
    params.set_parameter_float("sampling_time", dt)
    params.set_parameter_vector_string("states", ["WALKING", "STANCE"])

    params_walking = blf.parameters_handler.StdParametersHandler()
    params_stance = blf.parameters_handler.StdParametersHandler()

    params_walking.set_parameter_string("name", "walking")
    params_walking.set_parameter_vector_float("weight", walking_weight)

    params_stance.set_parameter_string("name", "stance")
    params_stance.set_parameter_vector_float("weight", stance_weight)

    assert params.set_group("WALKING", params_walking)
    assert params.set_group("STANCE", params_stance)

    provider = blf.continous_dynamical_system.MultiStateWeightProvider()
    assert provider.initialize(params)

    assert np.isclose(provider.get_weight(), walking_weight)
    assert provider.set_state("stance")

    for i in range(50):
        assert provider.advance()

    assert np.isclose(provider.get_weight(), stance_weight, rtol=tolerance)

    # reset the provider
    provider.reset("stance")
    assert np.isclose(provider.get_weight(), stance_weight)

    assert provider.set_state("walking")

    for i in range(50):
        assert provider.advance()

    assert np.isclose(provider.get_weight(), walking_weight, rtol=tolerance)
