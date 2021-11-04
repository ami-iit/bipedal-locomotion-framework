/**
 * @file Wrench.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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
