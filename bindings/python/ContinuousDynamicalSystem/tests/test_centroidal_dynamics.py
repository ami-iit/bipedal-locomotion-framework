import pytest
pytestmark = pytest.mark.centroidalDynamics

import bipedal_locomotion_framework.bindings as blf
import numpy as np
import manifpy as manif

from datetime import timedelta


def test_centroidal_dynamics():

    centroidal_integrator = blf.continuous_dynamical_system.CentroidalDynamicsForwardEulerIntegrator()
    centroidal_dynamics = blf.continuous_dynamical_system.CentroidalDynamics()
    dT = timedelta(milliseconds=100)
    assert centroidal_integrator.set_dynamical_system(centroidal_dynamics)
    assert centroidal_dynamics.set_state((np.zeros(3),np.zeros(3),np.zeros(3)))
    assert centroidal_integrator.set_integration_step(timedelta(milliseconds=100))

    centroidal_integrator.integrate(timedelta(0),dT)
