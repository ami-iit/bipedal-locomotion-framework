/**
 * @file ContactWrenchCone.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_CONTACT_WRENCH_CONE_H
#define BIPEDAL_LOCOMOTION_MATH_CONTACT_WRENCH_CONE_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>


namespace BipedalLocomotion
{
namespace Math
{

/**
 * ContactWrenchCone computes the polyhedral approximation of the contact wrench friction cone.
 * A surface remains fixed with respect to the environment if the contact wrench lies in a cone
 * described by
 * \f[
 * f ^c \cdot n > 0 \quad | f ^t | \le \mu f^c \cdot n
 * \f]
 *  where \f$ f^c \f$ is the
 * contact force, \f$ n \f$ is the vector normal to the contact surface. \f$ f^t \f$ is the
 * tangential force to the contact surface and \f$ \mu \f$ is the friction parameter. In addition we
 * require CoP inside the support area and a bounded yaw torque. The ContactWrenchCone aims to
 * compute the polyhedral approximation of \f$ | f ^t | \le \mu f^c \cdot n \f$ by splitting the base
 * of the cone into slices while considering the contact torque constraints. The class implements
 * the equations presented in [_Stability of Surface Contacts for Humanoid Robots: Closed-Form
 * Formulae of the Contact Wrench Cone for Rectangular Support Areas_
 * paper](https://ieeexplore.ieee.org/document/7139910). However differently from the original
 * work, the origin of the frame attached to the contact surface is not required to be in the
 * center of the surface.
 * @note The ContactWrenchCone class express the constraints in body representation (so called left
 * trivialization). Please check
 * [here](https://pure.tue.nl/ws/files/25753352/Traversaro_en_Saccon_DC_2016.064.pdf) for further
 * details. If you want to express the constraint in mixed representation please remember to
 * postmultiply the matrix A for the corresponding adjoint matrix. Namely:
 * \f[
 * A_{\text{mixed}} = A
 * \begin{bmatrix}
 * {}^I R _ B ^\top & 0 \\
 * 0 & {}^I R _ B^\top
 * \end{bmatrix}
 * \f]
 * where \f$I\f$ and \f$B\f$ are the inertial and the frame attached to the contact surface
 * respectively.
 * @note If you want to specify only the constraints related to the contact force please take a look
 * at LinearizedFrictionCone class.
 * @warning ContactWrenchCone class does not consider the unilaterally constraint of the normal
 * force.
 */
class ContactWrenchCone
{
    Eigen::MatrixXd m_A;
    Eigen::VectorXd m_b;

    bool m_isIntialized{false}; /**< True if the class has been correctly initialize */

public:

    /**
     * Initialize the Contact Wrench Cone class.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |         Parameter Name        |       Type       |                               Description                               | Mandatory |
     * |:-----------------------------:|:----------------:|:-----------------------------------------------------------------------:|:---------:|
     * |       `number_of_slices`      |       `int`      |                  Number of slices used to split 90 deg.                 |    Yes    |
     * | `static_friction_coefficient` |     `double`     |                       Static friction coefficient.                      |    Yes    |
     * |        `foot_limits_x`        | `vector<double>` | x coordinate of the foot limits w.r.t the frame attached to the surface |    Yes    |
     * |        `foot_limits_y`        | `vector<double>` | y coordinate of the foot limits w.r.t the frame attached to the surface |    Yes    |
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

#endif // BIPEDAL_LOCOMOTION_MATH_CONTACT_WRENCH_CONE_H
