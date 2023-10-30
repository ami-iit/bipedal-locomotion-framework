/**
 * @file Wrench.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_WRENCH_H
#define BIPEDAL_LOCOMOTION_MATH_WRENCH_H

#include <Eigen/Dense>

#include <manif/SE3.h>
#include <manif/SO3.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * Wrench represent a wrench, i.e. a 6d combination of linear force and angular torque.
 */
template <class Scalar> class Wrench : public Eigen::Matrix<Scalar, 6, 1>
{
public:
    using Base = Eigen::Matrix<Scalar, 6, 1>;

    /**
     * Inherit Eigen constructors
     */
    using Base::Base;

    /**
     * Get the linear force of the wrench
     * @return the sub-block containing the force
     */
    constexpr Eigen::Block<const Base, 3, 1> force() const
    {
        return this->Base::template head<3>();
    }

    /**
     * Get the angular torque of the wrench
     * @return the sub-block containing the torque
     */
    constexpr Eigen::Block<const Base, 3, 1> torque() const
    {
        return this->Base::template tail<3>();
    }

    /**
     * Get the linear force part of the wrench
     * @return the sub-block containing the force
     */
    Eigen::Block<Base, 3, 1> force()
    {
        return this->Base::template head<3>();
    }

    /**
     * Get the angular torque part of the wrench
     * @return the sub-block containing the torque
     */
    Eigen::Block<Base, 3, 1> torque()
    {
        return this->Base::template tail<3>();
    }

    /**
     * Get the local CoP associated with the wrench.
     * The center of pressure (CoP) is defined is a dynamic point defined in between two
     * bodies in contact. The CoP is a local quantity defined from interaction forces at the contact
     * surface.
     * @note A more detailed explanation can be found in: P. Sardain and G. Bessonnet, "Forces
     * acting on a biped robot. Center of pressure-zero moment point," in IEEE Transactions on
     * Systems, Man, and Cybernetics - Part A: Systems and Humans, vol. 34, no. 5, pp. 630-637,
     * Sept. 2004, doi: 10.1109/TSMCA.2004.832811. An interested reader can also check the following
     * https://scaron.info/robotics/zero-tilting-moment-point.html
     * @warning This function assumes that the wrench is expressed in the body frame. I.e., a frame
     * attached to the body belonging to the contact area and the z-component of the contact force
     * is the resultant pressure force.
     * @warning This function assumes that the z-component of the contact force is non-zero.
     * @return the CoP expressed in the local frame
     */
    Eigen::Matrix<Scalar, 3, 1> getLocalCoP() const
    {
        Eigen::Matrix<Scalar, 3, 1> localCoP;
        localCoP[0] = -this->torque()[1] / this->force()[2];
        localCoP[1] = this->torque()[0] / this->force()[2];
        localCoP[2] = Scalar(0);
        return localCoP;
    }
};

/**
 * Change the frame in which the Wrench is expressed.
 * If the tranform is \f$H\f$ is
 * \f[
 * (p,R) = ( {}^{\texttt{refOrient}} p_{\texttt{refPoint},\texttt{point}} , {}^{\texttt{refOrient}}
 * R_{\texttt{orient}} ) \f]
 *
 * And the wrench is:
 * \f[
 * {}_{\texttt{frame}} F =
 * \begin{bmatrix}
 * f \\ \tau
 * \end{bmatrix}
 * \f]
 *
 * The result of this operation is :
 * \f[
 * {}_{\texttt{refFrame}} F
 * =
 * {}_{\texttt{refFrame}}X^{\texttt{frame}}
 * {}_{\texttt{frame}} F
 * =
 * \begin{bmatrix}
 *    R & 0_{3\times3} \\
 *    p \times R &  R
 * \end{bmatrix}
 * \begin{bmatrix}
 * f \\ \tau
 * \end{bmatrix}
 * = \begin{bmatrix}
 * Rf \\ p \times R f + R\tau
 * \end{bmatrix}
 * \f]
 * @param transform a manif::SE3 object representing a homogeneous transformation that changes the
 * coordinates of a vector expressed in the frame A into the coordinates in the frame B
 * @param wrench expressed in the frame A
 * @return the vector expressed in the frame B
 */
template <class Scalar>
Wrench<Scalar> operator*(const manif::SE3<Scalar>& transform, const Wrench<Scalar>& wrench)
{
    Wrench<Scalar> ret;
    ret.force() = transform.rotation() * wrench.force();
    ret.torque() = transform.rotation() * wrench.torque() //
                   + transform.translation().cross(transform.rotation() * wrench.force());
    return ret;
}

/**
 * Change the frame in which the Wrench is expressed.
 * If the tranform is \f$H\f$ is
 * \f[
 * (p,R) = ( O _ {3 \times 1} , {}^{\texttt{refOrient}} R_{\texttt{orient}} )
 * \f]
 *
 * And the wrench is:
 * \f[
 * {}_{\texttt{frame}} F =
 * \begin{bmatrix}
 * f \\ \tau
 * \end{bmatrix}
 * \f]
 *
 * The result of this operation is :
 * \f[
 * {}_{\texttt{refFrame}} F
 * =
 * {}_{\texttt{refFrame}}X^{\texttt{frame}}
 * {}_{\texttt{frame}} F
 * =
 * \begin{bmatrix}
 *    R & 0_{3\times3} \\
 *    0_{3\times3} &  R
 * \end{bmatrix}
 * \begin{bmatrix}
 * f \\ \tau
 * \end{bmatrix}
 * = \begin{bmatrix}
 * Rf \\  R\tau
 * \end{bmatrix}
 * \f]
 * @param rotation a manif::SO3 object representing a rotation that changes the coordinates of a
 * vector expressed in the frame A into the coordinates in the frame B
 * @param wrench expressed in the frame A
 * @return the vector expressed in the frame B
 */
template <class Scalar>
Wrench<Scalar> operator*(const manif::SO3<Scalar>& rotation, const Wrench<Scalar>& wrench)
{
    Wrench<Scalar> ret;
    ret.force() = rotation.act(wrench.force());
    ret.torque() = rotation.act(wrench.torque());
    return ret;
}

/**
 * A wrench of double
 */
using Wrenchd = Wrench<double>;

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_WRENCH_H
