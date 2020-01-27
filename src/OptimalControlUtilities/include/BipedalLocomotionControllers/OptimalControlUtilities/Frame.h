/**
 * @file Frame.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H

#include <memory>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Model/Indices.h>

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * The class frame defines a Frame in a ControlProblemElement
 * @tparam T type of the frame identifier in the variable handler
 * @tparam U type of the frame identifier in the variable robot model
 */
template <typename T, typename U> class Frame
{
    /** Useful for identifying the variable in the handler (it can be also used as label) */
    T m_identifierInVariableHandler;

    U m_identifierInModel; /**< Useful for identifying the variable in the Model */

public:
    /**
     * Default constructor
     */
    Frame() = default;

    /**
     * Constructor
     * @param identifierInVariableHandler parameter used to identify the variable in the handler
     * @param identifierInModel parameter used to identify the variable in the model
     */
    Frame(const T& identifierInVariableHandler, const U& identifierInModel) noexcept;

    /**
     * Get the variable handler identifier
     * @return a reference to the parameter
     */
    T& identifierInVariableHandler() noexcept;

    /**
     * Get the variable handler identifier
     * @return a const reference to the parameter
     */
    const T& identifierInVariableHandler() const noexcept;

    /**
     * Get the model identifier
     * @return a reference to the parameter
     */
    U& identifierInModel() noexcept;

    /**
     * Get the model identifier
     * @return a const reference to the parameter
     */
    const U& identifierInModel() const noexcept;
};

/**
 * The class frame defines a Frame in contact with the environment
 * @tparam T type of the frame identifier in the variable handler
 * @tparam U type of the frame identifier in the variable robot model
 */
template <typename T, typename U> class FrameInContact : public Frame<T, U>
{
    bool m_isInCompliantContact{false}; /** If the contact with the environment is described using a
                                           compliant model the parameter is true */

public:
    using Frame<T, U>::Frame;

    /**
     * Constructor
     * @param identifierInVariableHandler parameter used to identify the variable in the handler
     * @param identifierInModel parameter used to identify the variable in the model
     * @param isInCompliantContact true if the contact is described using a compliant contact model
     */
    FrameInContact(const T& identifierInVariableHandler,
                   const U& identifierInModel,
                   const bool& isInCompliantContact) noexcept;

    /**
     * Get the compliant contact state
     * @return a reference to the parameter
     */
    bool& isInCompliantContact() noexcept;

    /**
     * Get the compliant contact state
     * @return a const reference to the parameter
     */
    const bool& isInCompliantContact() const noexcept;
};

/**
 * The class frame defines a Frame in contact with the environment with the wrench value
 * @tparam T type of the frame identifier in the variable handler
 * @tparam U type of the frame identifier in the variable robot model
 */
template <typename T, typename U> class FrameInContactWithWrench : public FrameInContact<T, U>
{
    /** Contact wrench expressed in mixed representation */
    iDynTree::Wrench m_contactWrench{iDynTree::Wrench::Zero()};

public:
    using FrameInContact<T, U>::FrameInContact;

    /**
     * Get the contact wrench
     * @return a reference to the parameter
     */
    iDynTree::Wrench& contactWrench() noexcept;

    /**
     * Get the contact wrench
     * @return a const reference to the parameter
     */
    const iDynTree::Wrench& contactWrench() const noexcept;
};

/**
 * The class frame defines a Frame in contact with the environment and its associated contact model
 * @tparam T type of the frame identifier in the variable handler
 * @tparam U type of the frame identifier in the variable robot model
 */
template<typename T, typename U>
class FrameInContactWithContactModel : public FrameInContact<T, U>
{
    /** The contact model sued to describe the interaction between the link and the environment */
    std::shared_ptr<ContactModels::ContactModel> m_contactModel{nullptr};

public:
    using FrameInContact<T, U>::FrameInContact;

    FrameInContactWithContactModel(
        const T& identifierInVariableHandler,
        const U& identifierInModel,
        const bool& isInCompliantContact,
        std::shared_ptr<ContactModels::ContactModel> contactModel) noexcept;

    FrameInContactWithContactModel(
        const T& identifierInVariableHandler,
        const U& identifierInModel,
        std::shared_ptr<ContactModels::ContactModel> contactModel) noexcept;


    std::shared_ptr<ContactModels::ContactModel>& contactModel() noexcept;
    const std::shared_ptr<ContactModels::ContactModel>& contactModel() const noexcept;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#include "Frame.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H
