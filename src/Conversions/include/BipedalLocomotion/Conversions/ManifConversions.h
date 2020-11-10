/**
 * @file Conversions.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H
#define BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <manif/manif.h>


namespace BipedalLocomotion
{
namespace Conversions
{
    /**
     * @brief Convert rotation matrix and translation vector to manif SE3 object
     *
     * @param rotation reference to 3x3 Eigen matrix
     * @param translation reference of 3x1 Eigen matrix
     * @return pose as manif SE3 object
     */
    template <class Scalar>
    manif::SE3<Scalar> toManifPose(const Eigen::Matrix<Scalar, 3, 3>& rotation,
                                   const Eigen::Matrix<Scalar, 3, 1>& translation)
    {
        Eigen::Quaternion<Scalar> quat = Eigen::Quaternion<Scalar>(rotation);
        quat.normalize(); // SO3 constructor expects normalized quaternion
        return manif::SE3<Scalar>(translation, quat);
    }

    /**
     * @brief Convert rotation matrix and translation vector to manif SE3d object
     *
     * @param rotation Eigen ref of 3x3 rotation matrix
     * @param translation Eigen ref of 3x1 translation vector
     * @return pose as manif SE3d object
     */
    inline manif::SE3d toManifPose(Eigen::Ref<const Eigen::Matrix3d> rotation,
                                   Eigen::Ref<const Eigen::Vector3d> translation)
    {
        Eigen::Quaterniond quat = Eigen::Quaterniond(rotation);
        quat.normalize(); // SO3 constructor expects normalized quaternion
        return manif::SE3d(translation, quat);
    }

    /**
     * @brief Convert iDynTree transform object to manif SE3d object
     *
     * @param H reference to iDynTree Tranform object
     * @return pose as manif SE3d object
     */
    inline manif::SE3d toManifPose(const iDynTree::Transform& H)
    {
        return toManifPose(iDynTree::toEigen(H.getRotation()),
                           iDynTree::toEigen(H.getPosition()));;
    }

} // namespace Conversions
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H
