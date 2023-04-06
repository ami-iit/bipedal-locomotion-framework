import pytest

import bipedal_locomotion_framework.bindings as blf
import numpy as np
import manifpy as manif

from datetime import timedelta

def close_form_solution(t, position0, rotation0, joint_position0, twist, joint_velocity):
    return position0 + t * twist[0:3], \
        manif.SO3Tangent(twist[3:6] *t) + rotation0, \
        joint_position0 + t * joint_velocity

def test_floating_base_system_kinematics():

    tolerance = 1e-3
    dt = 0.0001
    simulation_time = 0.5

    np.random.seed(42)
    twist = np.random.uniform(-2, 2, (6))

    joint_velocity = np.random.uniform(-2, 2, (23))

    rotation0 = manif.SO3.Identity()
    position0 = np.array([0,0,0])
    joint_position0 = np.zeros(23)


    system = blf.continuous_dynamical_system.FloatingBaseSystemKinematics()
    system.state = position0, rotation0, joint_position0
    assert system.set_control_input((twist, joint_velocity))

    integrator = blf.continuous_dynamical_system.FloatingBaseSystemKinematicsForwardEulerIntegrator()
    assert integrator.set_dynamical_system(system)
    integrator.integration_step = timedelta(seconds=dt)


    for i in range(0, int(simulation_time / dt)):
        base_position, base_rotation, joint_position = integrator.get_solution()
        base_position_exact, base_rotation_exact, joint_position_exact = close_form_solution(i * dt, position0, rotation0, joint_position0, twist, joint_velocity)


        assert base_position == pytest.approx(base_position_exact, abs=tolerance)
        assert base_rotation.rotation() == pytest.approx(base_rotation_exact.rotation(), abs=tolerance)
        assert joint_position == pytest.approx(joint_position_exact, abs=tolerance)

        assert integrator.integrate(timedelta(seconds=0), timedelta(seconds=dt))
