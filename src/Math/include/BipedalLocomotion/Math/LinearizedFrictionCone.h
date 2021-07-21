/**
 * @file LinearizedFrictionCone.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_LINEARIZED_FRICTION_CONE_H
#define BIPEDAL_LOCOMOTION_MATH_LINEARIZED_FRICTION_CONE_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * LinearizedFrictionCone computes the polyhedral approximation of the friction cone. A point contact remains
 * fixed with respect the environment if the contact force lies in a cone described by
 * \f[
 * f ^c \cdot n > 0 \quad | f ^t | \le \mu f^c \cdot n
 * \f]
 * where \f$ f^c \f$ is the contact force, \f$ n \f$ is the vector normal to the contact surface. \f$ f^t \f$ is
 * the tangential force to the contact surface and \f$ \mu \f$ is the friction parameter.
 * The LinearizedFrictionCone aims to compute the polyhedral approximation of \f$ | f ^t | \le \mu f^c \cdot n \f$
 * by spitting the base of the cone into slices.
 */
class LinearizedFrictionCone
{
    Eigen::MatrixXd m_A;
    Eigen::VectorXd m_b;

    bool m_isIntialized{false}; /**< True if the class has been correctly initialize */

public:

    /**
     * Initialize the continuous algebraic riccati equation solver.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |           Parameter Name        |    Type   |               Description               |
     * |:-------------------------------:|:---------:|:---------------------------------------:|
     * |        `number_of_slices`       |   `int`   |  Number of slices used to split 90 deg. |
     * | `static_friction_coefficient`   | `double`  |        Static friction coefficient.     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Get the matrix A.
     * @return the matrix A.
     */
    Eigen::Ref<const Eigen::MatrixXd> getA() const;

    /**
     * Get the vector B.
     * @return the matrix B..
     */
    Eigen::Ref<const Eigen::VectorXd> getB() const;
};

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_LINEARIZED_FRICTION_CONE_H
