import pytest

import bipedal_locomotion_framework.bindings as blf
import numpy as np

from datetime import timedelta

def close_form_solution(t):
    return np.array([1 - np.exp(-t) * (np.cos(t) + np.sin(t)), 2 * np.exp(-t) * np.sin(t)])

def test_linear_system():

    A = np.array([[0,1],[-2,-2]])
    B = np.array([0,2])
    u = np.array([1])
    x0 = np.array([0, 0])
    dt = 0.0001
    simulation_time = 1

    system = blf.continuous_dynamical_system.LinearTimeInvariantSystem()
    assert system.set_system_matrices(A,B)
    assert system.set_state((x0,))
    assert system.set_control_input((u,))

    integrator = blf.continuous_dynamical_system.LinearTimeInvariantSystemForwardEulerIntegrator()
    assert integrator.set_dynamical_system(system)
    assert integrator.set_integration_step(timedelta(seconds=dt))

    tolerance = 1e-3

    for i in range(0, int(simulation_time / dt)):
        solution, = integrator.get_solution()
        assert solution == pytest.approx(close_form_solution(i * dt), abs=tolerance)
        assert integrator.integrate(timedelta(seconds=0), timedelta(seconds=dt))
