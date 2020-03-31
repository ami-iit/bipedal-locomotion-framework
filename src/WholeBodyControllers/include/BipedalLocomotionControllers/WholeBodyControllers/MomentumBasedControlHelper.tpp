/**
 * @file MomentumBasedTorqueControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
template <OptimalControlUtilities::CartesianElementType type>
bool MomentumBasedControlHelper::addCartesianElement(
    std::shared_ptr<ParametersHandler::IParametersHandler> handler,
    const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;

    const auto& label = frame.identifierInVariableHandler();

    if (m_cartesianElements.find(label) != m_cartesianElements.end()
        || m_orientationElements.find(label) != m_orientationElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The element named " << label
                  << " has been already added." << std::endl;
        return false;
    }

    // initialize the PD controller
    typename CartesianElement<type>::ControllerType pdController;

    // if the type is Pose or orientation
    double kpRotational, kdRotational, c0;
    if constexpr (type == CartesianElementType::ORIENTATION || type == CartesianElementType::POSE)
    {
        if (!handler->getParameter("kp_rotational", kpRotational))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kp cannot be "
                         "found"
                      << std::endl;
            return false;
        }
        bool useDefaultKp = false;
        handler->getParameter("use_default_kd_rotational", useDefaultKp);
        if (useDefaultKp)
        {
            double scaling = 1.0;
            handler->getParameter("scaling_rotational", scaling);
            kdRotational = 2 / scaling * std::sqrt(kpRotational);
        } else
        {
            if (!handler->getParameter("kd_rotational", kdRotational))
            {
                std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kd cannot be "
                             "found"
                          << std::endl;
                return false;
            }
        }

        if (!handler->getParameter("c0", c0))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The c0 cannot be "
                         "found"
                      << std::endl;
            return false;
        }
    }

    // if the type is position or pose
    iDynTree::Vector3 kpPosition, kdPosition;
    if constexpr (type == CartesianElementType::POSITION || type == CartesianElementType::POSE)
    {
        if (!handler->getParameter("kp_position", kpPosition))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kp cannot be "
                         "found"
                      << std::endl;
            return false;
        }
        bool useDefaultKp = false;
        handler->getParameter("use_default_kd_position", useDefaultKp);
        if (useDefaultKp)
        {
            double scaling = 1.0;
            handler->getParameter("scaling_linear", scaling);
            iDynTree::toEigen(kdPosition)
                = 2 / scaling * iDynTree::toEigen(kpPosition).array().sqrt();
        } else
        {
            if (!handler->getParameter("kd_position", kdPosition))
            {
                std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kd cannot be "
                             "found"
                          << std::endl;
                return false;
            }
        }
    }

    // set the gains for each type of PD
    if constexpr (type == CartesianElementType::POSITION)
        pdController.setGains(kpPosition, kdPosition);

    if constexpr (type == CartesianElementType::ORIENTATION)
        pdController.setGains(c0, kdRotational, kpRotational);

    if constexpr (type == CartesianElementType::POSE)
        pdController.setGains(kpPosition, kdPosition, c0, kdRotational, kpRotational);

    typename dictionary<unique_ptr<CartesianElement<type>>>::iterator element;

    // initialize the element
    if constexpr (type == CartesianElementType::ORIENTATION)
    {
        m_orientationElements
            .emplace(label,
                     std::make_unique<CartesianElement<type>>(m_kinDyn,
                                                              pdController,
                                                              m_variableHandler,
                                                              frame.identifierInModel()));
        element = m_orientationElements.find(label);
    } else if constexpr (type == CartesianElementType::POSE)
    {
        m_cartesianElements
            .emplace(label,
                     std::make_unique<CartesianElement<type>>(m_kinDyn,
                                                              pdController,
                                                              m_variableHandler,
                                                              frame.identifierInModel()));

        element = m_cartesianElements.find(label);
    }

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight",
                                   rawWeight,
                                   GenericContainer::VectorResizeMode::Resizable))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] Unable to get the "
                         "Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(element->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_cartesian_element");
    } else
        m_constraints->addConstraint(element->second.get());

    return true;
}

} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers
