/**
 * @file Advanceable.h
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H

#include <variant>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/InputPort.h>
#include <BipedalLocomotion/System/OutputPort.h>

namespace BipedalLocomotion
{
namespace System
{
/**
 * Type that can be used to described an empty signal. It should be used with advanceable if Input
 * or Output are not required.
 */
using EmptySignal = std::monostate;

template <class _Input, class _Output> class Advanceable;
} // namespace System
} // namespace BipedalLocomotion

/**
 * Basic class that represents a discrete system. The interface contains method to set inputs and
 * output.
 */
template <class _Input, class _Output>
class BipedalLocomotion::System::Advanceable : public BipedalLocomotion::System::InputPort<_Input>,
                                               public BipedalLocomotion::System::OutputPort<_Output>
{
public:
    using Input = typename BipedalLocomotion::System::InputPort<_Input>::Input;
    using Output = typename BipedalLocomotion::System::OutputPort<_Output>::Output;

    Advanceable() = default;
    Advanceable(const Advanceable&) = delete;
    Advanceable(Advanceable&&) = delete;
    Advanceable& operator=(const Advanceable&) = delete;
    Advanceable& operator=(Advanceable&&) = delete;
    virtual ~Advanceable() = default;

    /**
     * @brief Initialize the advanceable
     * @note the default implementation does nothing.
     * @return True if the initialization is successfull.
     */
    virtual bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * @brief Advance the internal state. This may change the value retrievable from getOutput().
     * @return True if the advance is successfull.
     */
    virtual bool advance() = 0;

    /**
     * @brief Close the Advanceable
     * @note the default implementation does nothing.
     * @return True if the close is successfull.
     */
    virtual bool close();
};

namespace BipedalLocomotion
{
namespace System
{

template <class _Input, class _Output> bool Advanceable<_Input, _Output>::close()
{
    return true;
}

template <class _Input, class _Output>
bool Advanceable<_Input, _Output>::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    return true;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H
