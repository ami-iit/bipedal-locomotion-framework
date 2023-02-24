import pytest
pytestmark = pytest.mark.conversions

import bipedal_locomotion_framework.bindings as blf
import manifpy as manif
import numpy as np

import idyntree.swig as idyn

def test_manif_conversions_from_numpy():
    idyntree_so3 = idyn.Rotation.RPY(0.1, 0.2, -1.32)
    numpy_so3 = idyntree_so3.toNumPy()
    manif_so3 = blf.conversions.to_manif_rot(numpy_so3)
    assert numpy_so3 == pytest.approx(manif_so3.rotation())

    traslation = [9.1, -1.2, 3.1]
    manif_se3 = blf.conversions.to_manif_pose(numpy_so3, traslation)
    assert numpy_so3 == pytest.approx(manif_se3.rotation())
    assert traslation == pytest.approx(manif_se3.translation())


def test_manif_conversions_from_idyntree():
    idyntree_so3 = idyn.Rotation.RPY(0.1, 0.2, -1.32)
    manif_so3 = blf.conversions.to_manif_rot(idyntree_so3)
    assert idyntree_so3.toNumPy() == pytest.approx(manif_so3.rotation())

    traslation = [9.1, -1.2, 3.1]
    idyntree_se3 = idyn.Transform(idyntree_so3, traslation)
    manif_se3 = blf.conversions.to_manif_pose(idyntree_se3)
    assert idyntree_se3.getRotation().toNumPy() == pytest.approx(manif_se3.rotation())
    assert idyntree_se3.getPosition().toNumPy() == pytest.approx(manif_se3.translation())
