/**
 * @file GlobalZMPEvaluator.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_ZMP_EVALUATOR_H
#define BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_ZMP_EVALUATOR_H

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
 * GlobalZMPEvaluator is a class that computes the global ZMP given a set of contact wrenches.
 * The ZMP is computed as the weighted average of the ZMP of each contact, with the weight
 * determined by the normal force of the contact.
 * @note This class assumes that the contact wrenches stored in Contacts::ContactWrench list are
 * expressed in the body frame (left trivialized).
 * @note In addition to evaluating the ZMP, this class checks that at least one contact is active
 * and that the ZMP is not constant for a given number of iterations. Moreover, the class verifies
 * that the local ZMP belongs to a given region; otherwise, the associated local ZMP is not
 * considered in the computation of the global ZMP.
 */
class GlobalZMPEvaluator
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
     * |   `zmp_admissible_limits`  | `vector<double>` |  2D vector defines ZMP region, comparing absolute ZMP x and y to the 1st and 2nd elements |    Yes    |
     * |  `constant_zmp_tolerance`  |     `double`     |        Radius (in m) of a sphere used to considered if the global ZMP is constant         |    Yes    |
     * | `constant_zmp_max_counter` |      `int`       |         Maximum number of samples after which a constant ZMP generates an error           |    Yes    |
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
     * Compute the global ZMP.
     * @return true in case of success and false otherwise
     */
    bool advance() override;

    /**
     * Check if the ZMP evaluated by the class is valid.
     * @return true if valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the global ZMP
     * @return a 3D vector containing the position of the global ZMP expressed respect to the global
     * (inertial) frame.
     */
    const Eigen::Vector3d& getOutput() const override;

private:
    Eigen::Vector3d m_zmp{Eigen::Vector3d::Zero()}; /**< Global ZMP position in the inertial frame
                                                     */
    std::vector<Contacts::ContactWrench> m_contacts; /**< */
    bool m_isInitialized{false}; /**< True if the object is initialized */
    bool m_isOutputValid{false}; /**< True if the output is valid */

    int m_constantZMPCounter{0}; /**< Counter used to store the number of constant ZMP over time */

    Eigen::Vector2d m_zmpAdmissibleLimits; /**< Vector containing the local admissible limits for
                                              the ZMP  */
    int m_constantZMPMaxCounter{-1}; /**< Maximum number of samples after which a constant ZMP
                                        generates an error   */
    double m_minimumNormalForce{0.0}; /**< Minimum required contact force */
    double m_constantZMPTolerance{0.0}; /**< Radius (in m) of a sphere used to considered if the
                                           global ZMP is constant */
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACTS_GLOBAL_ZMP_EVALUATOR_H
