/**
 * @file ManifConversions.h
 * @authors Prashanth Ramadoss Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H
#define BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>

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
    return toManifPose(iDynTree::toEigen(H.getRotation()), iDynTree::toEigen(H.getPosition()));
}

/**
 * @brief Convert rotation matrix to manif SO3 object
 *
 * @param rotation reference to 3x3 Eigen matrix
 * @return pose as manif SO3 object
 */
template <class Scalar> manif::SO3<Scalar> toManifRot(const Eigen::Matrix<Scalar, 3, 3>& rotation)
{
    Eigen::Quaternion<Scalar> quat = Eigen::Quaternion<Scalar>(rotation);
    quat.normalize(); // SO3 constructor expects normalized quaternion
    return manif::SO3<Scalar>(quat);
}

/**
 * @brief Convert rotation matrix to manif SO3d object
 *
 * @param rotation Eigen ref of 3x3 rotation matrix
 * @param translation Eigen ref of 3x1 translation vector
 * @return pose as manif SO3d object
 */
inline manif::SO3d toManifRot(Eigen::Ref<const Eigen::Matrix3d> rotation)
{
    Eigen::Quaterniond quat = Eigen::Quaterniond(rotation);
    quat.normalize(); // SO3 constructor expects normalized quaternion
    return manif::SO3d(quat);
}

/**
 * @brief Convert iDynTree rotation object to manif SE3d object
 *
 * @param R reference to iDynTree rotation object
 * @return pose as manif SO3d object
 */
inline manif::SO3d toManifRot(const iDynTree::Rotation& R)
{
    return toManifRot(iDynTree::toEigen(R));
}

/**
 * @brief Convert iDynTree twist object to manif SE3Tangentd object
 *
 * @param twist reference to iDynTree twist object
 * @return a twist as manif SE3Tangentd object
 */
inline manif::SE3Tangentd toManifTwist(const iDynTree::Twist& twist)
{
    manif::SE3Tangentd manifTwist;
    manifTwist.lin() = iDynTree::toEigen(twist.getLinearVec3());
    manifTwist.ang() = iDynTree::toEigen(twist.getAngularVec3());
    return manifTwist;
}

/**
 * @brief Convert a manif SE3 object into and iDynTree::Transform
 *
 * @param se3 a manif SE3 object
 * @return pose as iDynTree::Transform
 */
inline iDynTree::Transform toiDynTreePose(const manif::SE3d& se3)
{
    return iDynTree::Transform(iDynTree::make_matrix_view(se3.transform()));
}

/**
 * @brief Convert a manif SO3 object into and iDynTree::Rotation
 *
 * @param so3 a manif SO3 object
 * @return rotation as iDynTree::Rotation
 */
inline iDynTree::Rotation toiDynTreeRot(const manif::SO3d& so3)
{
    return iDynTree::Rotation(iDynTree::make_matrix_view(so3.rotation()));
}

} // namespace Conversions
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MANIF_CONVERSIONS_H
