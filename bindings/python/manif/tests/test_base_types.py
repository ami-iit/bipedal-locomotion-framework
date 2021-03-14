import pytest
pytestmark = pytest.mark.planners

import bipedal_locomotion_framework.bindings as blf
import numpy as np


def test_se3():

    # Test the default constructor
    transform_default = blf.manif.SE3()
    assert transform_default.position == pytest.approx(np.array([0, 0, 0]))
    assert transform_default.quaternion == pytest.approx(np.array([0, 0, 0, 1]))

    # Test the custom constructor
    transform = blf.manif.SE3(position=np.array([0, 1, 2.]),
                              quaternion=np.array([0, 1., 0, 0.]))
    assert transform.position == pytest.approx([0, 1, 2])
    assert transform.quaternion == pytest.approx([0, 1., 0, 0.])

    # Test the position setter
    transform.position = np.array([-1, 42.0, 3.14])
    assert transform.position == pytest.approx([-1, 42.0, 3.14])

    # Create a quaternion
    quaternion_not_normalized = np.array([1, 1, 0, 0])
    quaternion = quaternion_not_normalized / np.linalg.norm(quaternion_not_normalized)

    # Test the quaternion setter
    transform.quaternion = quaternion
    assert transform.quaternion == pytest.approx(quaternion)

    # Test equality operator
    assert transform == blf.manif.SE3(position=[-1, 42.0, 3.14], quaternion=quaternion)
