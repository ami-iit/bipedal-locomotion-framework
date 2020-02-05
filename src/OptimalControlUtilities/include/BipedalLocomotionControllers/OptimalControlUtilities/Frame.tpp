/**
 * @file Frame.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

template <typename T, typename U>
Frame<T, U>::Frame(const T& identifierInVariableHandler, const U& identifierInModel) noexcept
    : m_identifierInVariableHandler(identifierInVariableHandler)
    , m_identifierInModel(identifierInModel)
{
}

template <typename T, typename U> U& Frame<T, U>::identifierInModel() noexcept
{
    return m_identifierInModel;
}

template <typename T, typename U> const U& Frame<T, U>::identifierInModel() const noexcept
{
    return m_identifierInModel;
}

template <typename T, typename U> T& Frame<T, U>::identifierInVariableHandler() noexcept
{
    return m_identifierInVariableHandler;
}

template <typename T, typename U> const T& Frame<T, U>::identifierInVariableHandler() const noexcept
{
    return m_identifierInVariableHandler;
}

template <typename T, typename U>
FrameInContact<T, U>::FrameInContact(const T& identifierInVariableHandler,
                                     const U& identifierInModel,
                                     const bool& isInCompliantContact) noexcept
    : Frame<T, U>{identifierInVariableHandler, identifierInModel}
    , m_isInCompliantContact{isInCompliantContact}
{
}

template <typename T, typename U>
const bool& FrameInContact<T, U>::isInCompliantContact() const noexcept
{
    return m_isInCompliantContact;
}

template <typename T, typename U> bool& FrameInContact<T, U>::isInCompliantContact() noexcept
{
    return m_isInCompliantContact;
}

template <typename T, typename U>
const iDynTree::Wrench& FrameInContactWithWrench<T, U>::contactWrench() const noexcept
{
    return m_contactWrench;
}


template <typename T, typename U>
iDynTree::Wrench& FrameInContactWithWrench<T, U>::contactWrench() noexcept
{
    return m_contactWrench;
}

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers
