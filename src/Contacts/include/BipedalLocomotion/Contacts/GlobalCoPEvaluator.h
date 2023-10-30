/**
 * @file GlobalCoPEvaluator.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_COP_EVALUATOR_H
#define BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_COP_EVALUATOR_H

#include <initializer_list>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * GlobalCoPEvaluator is a class that computes the global CoP given a set of contact wrenches.
 * Following P. Sardain and G. Bessonnet, "Forces acting on a biped robot. Center of
 * pressure-zero moment point," in IEEE Transactions on Systems, Man, and Cybernetics,
 * we defined the global CoP as the weighted average of the CoP of each contact, with the weight
 * determined by the normal force of the contact.
 * @note This class assumes that the contact wrenches stored in Contacts::ContactWrench list are
 * expressed in the body frame (left trivialized).
 * @note This class assumes that all the contacts are coplanar, that the normal of the common
 * plane is the z axis, and that the contact force on the z axis is positive.
 * @note In addition to evaluating the CoP, this class checks that at least one contact is active
 * and that the CoP is not constant for a given number of iterations.
 */
class GlobalCoPEvaluator
    : public BipedalLocomotion::System::Advanceable<std::vector<Contacts::ContactWrench>,
                                                    Eigen::Vector3d>
{
public:
    // clang-format off
    /**
     * Initialize the class
     * @param handler pointer to a parameter handler
     * @note the following parameters are required by the class
     * |       Parameter Name       |       Type       |                                           Description                                     | Mandatory |
     * |:--------------------------:|:----------------:|:-----------------------------------------------------------------------------------------:|:---------:|
     * |   `minimum_normal_force`   |     `double`     |           Minimum normal force required to consider a contact active (in N)               |    Yes    |
     * |   `cop_admissible_limits`  | `vector<double>` |  2D vector defines CoP region, comparing absolute CoP x and y to the 1st and 2nd elements |    Yes    |
     * |  `constant_cop_tolerance`  |     `double`     |        Radius (in m) of a sphere used to considered if the global CoP is constant         |    Yes    |
     * | `constant_cop_max_counter` |      `int`       |         Maximum number of samples after which a constant CoP generates an error           |    Yes    |
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the input to the class.
     * @param input list containing the contact wrench
     * @return true in case of success and false otherwise
     * @note This class assumes that the contact wrenches stored in Contacts::ContactWrench list are
     * expressed in the body frame (left trivialized).
     */
    bool setInput(const std::initializer_list<Contacts::ContactWrench>& input);

    /**
     * Set the input to the class.
     * @param input list containing the contact wrench
     * @return true in case of success and false otherwise
     * @note This class assumes that the contact wrenches stored in Contacts::ContactWrench list are
     * expressed in the body frame (left trivialized).
     */
    bool setInput(const std::vector<Contacts::ContactWrench>& input) override;

    /**
     * Compute the global CoP.
     * @return true in case of success and false otherwise
     */
    bool advance() override;

    /**
     * Check if the CoP evaluated by the class is valid.
     * @return true if valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the global CoP
     * @return a 3D vector containing the position of the global CoP expressed respect to the global
     * (inertial) frame.
     */
    const Eigen::Vector3d& getOutput() const override;

private:
    Eigen::Vector3d m_cop{Eigen::Vector3d::Zero()}; /**< Global CoP position in the inertial frame
                                                     */
    std::vector<Contacts::ContactWrench> m_contacts; /**< Vector containing the contacts */
    bool m_isInitialized{false}; /**< True if the object is initialized */
    bool m_isOutputValid{false}; /**< True if the output is valid */

    int m_constantCoPCounter{0}; /**< Counter used to store the number of constant CoP over time */

    Eigen::Vector2d m_CoPAdmissibleLimits; /**< Vector containing the local admissible limits for
                                              the CoP  */
    int m_constantCoPMaxCounter{-1}; /**< Maximum number of samples after which a constant CoP
                                        generates an error   */
    double m_minimumNormalForce{0.0}; /**< Minimum required contact force */
    double m_constantCoPTolerance{0.0}; /**< Radius (in m) of a sphere used to considered if the
                                           global CoP is constant */
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_COP_EVALUATOR_H
