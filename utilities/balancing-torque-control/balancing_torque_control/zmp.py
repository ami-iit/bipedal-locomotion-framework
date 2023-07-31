# This software may be modified and distributed under the terms of the BSD-3-Clause license.
# Authors: Giulio Romualdi

import idyntree.swig as idyn


def evaluate_local_zmp(wrench, contact_force_threshold):
    tau_x = wrench[3]
    tau_y = wrench[4]
    f_z = wrench[2]
    if f_z >= contact_force_threshold:
        return [-tau_y / f_z, tau_x / f_z, 0.0], True
    return [0.0, 0.0, 0.0], False


def evaluate_global_zmp(
    left_wrench,
    right_wrench,
    kindyn: idyn.KinDynComputations,
    l_sole_frame,
    r_sole_frame,
    contact_force_threshold,
):
    def to_int(is_defined):
        if is_defined:
            return 1
        return 0

    left_zmp, zmp_left_defined = evaluate_local_zmp(
        left_wrench, contact_force_threshold
    )
    right_zmp, zmp_right_defined = evaluate_local_zmp(
        right_wrench, contact_force_threshold
    )

    total_z = right_wrench[2] * to_int(zmp_right_defined) + left_wrench[2] * to_int(
        zmp_left_defined
    )

    inertial_zmp_left = kindyn.getWorldTransform(l_sole_frame) * idyn.Position(left_zmp)
    inertial_zmp_right = kindyn.getWorldTransform(r_sole_frame) * idyn.Position(
        right_zmp
    )

    inertial_global_zmp = (
        left_wrench[2]
        * to_int(zmp_left_defined)
        * inertial_zmp_left.toNumPy()
        / total_z
        + right_wrench[2]
        * to_int(zmp_right_defined)
        * inertial_zmp_right.toNumPy()
        / total_z
    )

    return inertial_global_zmp
