/**
 * @file Conversions.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_COMMON_CONVERSIONS_H
#define BIPEDAL_LOCOMOTION_COMMON_CONVERSIONS_H

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace Conversions
{
    /**
     * @brief Construct homogeneous transformation matrix from rotation matrix and translation vector
     *
     * @param rotation reference of rotation matrix
     * @param translation reference of translation vector
     * @return homogeneous transform as a Eigen::Matrix4d object
     */
    template <class Scalar>
    Eigen::Matrix<Scalar, 4, 4> toEigenPose(const Eigen::Matrix<Scalar, 3, 3>& rotation,
                                            const Eigen::Matrix<Scalar, 3, 1>& translation)
    {
        Eigen::Matrix<Scalar, 4, 4> transform;
        transform.template topLeftCorner<3,3>() = rotation;
        transform.template topRightCorner<3,1>() = translation;
        transform(3,3) = 1;
        transform.template bottomLeftCorner<1,3>().setZero();
        return transform;
    }


} // namespace Conversions
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_COMMON_CONVERSIONS_H
