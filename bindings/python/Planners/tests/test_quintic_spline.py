import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import numpy as np


def test_quintic_spline():

    number_of_points = 100
    init_time = 0.32
    final_time = 2.64

    dT = (final_time - init_time) / (number_of_points - 1)

    coefficients = [np.random.rand(4, 1) for _ in range(6)]

    time = [init_time + dT * i for i in range(number_of_points)]

    knots = [
        coefficients[0] + coefficients[1] * t
        + coefficients[2] * (t ** 2)
        + coefficients[3] * (t ** 3)
        + coefficients[4] * (t ** 4)
        + coefficients[5] * (t ** 5) for t in time]

    init_velocity = \
        coefficients[1] + 2 \
        * coefficients[2] * init_time \
        + 3 * coefficients[3] * (init_time ** 2) \
        + 4 * coefficients[4] * (init_time ** 3) \
        + 5 * coefficients[5] * (init_time ** 4)

    init_acceleration = \
        2 * coefficients[2] \
        + 3 * 2 * coefficients[3] * init_time \
        + 4 * 3 * coefficients[4] * (init_time ** 2) \
        + 5 * 4 * coefficients[5] * (init_time ** 3)

    final_velocity = \
        coefficients[1] \
        + 2 * coefficients[2] * final_time \
        + 3 * coefficients[3] * (final_time ** 2) \
        + 4 * coefficients[4] * (final_time ** 3) \
        + 5 * coefficients[5] * (final_time ** 4)

    final_acceleration = \
        2 * coefficients[2] \
        + 3 * 2 * coefficients[3] * final_time \
        + 4 * 3 * coefficients[4] * (final_time ** 2) \
        + 5 * 4 * coefficients[5] * (final_time ** 3)

    spline = blf.planners.QuinticSpline()

    assert spline.set_knots(knots, time)
    assert spline.set_initial_conditions(init_velocity, init_acceleration)
    assert spline.set_final_conditions(final_velocity, final_acceleration)

    points_to_check_number = 1e4

    dT_check_points = (final_time - init_time) / points_to_check_number

    position = np.zeros(4)
    velocity = np.zeros(4)
    acceleration = np.zeros(4)

    for i in range(int(points_to_check_number)):

        t = dT_check_points * i + init_time

        assert spline.evaluate_point(t, position, velocity, acceleration)

        expected = \
            coefficients[0] \
            + coefficients[1] * t \
            + coefficients[2] * (t ** 2) \
            + coefficients[3] * (t ** 3) \
            + coefficients[4] * (t ** 4) \
            + coefficients[5] * (t ** 5)

        assert np.allclose(expected.transpose()[0], position, atol=1e-5)

        # check velocity
        expected = \
            coefficients[1] \
            + 2 * coefficients[2] * t \
            + 3 * coefficients[3] * (t ** 2) \
            + 4 * coefficients[4] * (t ** 3) \
            + 5 * coefficients[5] * (t ** 4)

        assert np.allclose(expected.transpose()[0], velocity, atol=1e-5)

        # check acceleration
        expected = \
            2 * coefficients[2] \
            + 3 * 2 * coefficients[3] * t \
            + 4 * 3 * coefficients[4] * (t ** 2) \
            + 5 * 4 * coefficients[5] * (t ** 3)

        assert np.allclose(expected.transpose()[0], acceleration, atol=1e-5)
