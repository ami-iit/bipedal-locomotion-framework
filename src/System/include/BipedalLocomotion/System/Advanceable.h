/**
 * @file Advanceable.h
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H

#include <variant>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

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
template <class _Input, class _Output> class BipedalLocomotion::System::Advanceable
{
public:
    using Input = _Input;
    using Output = _Output;

    /**
     * @brief Initialize the advanceable
     * @note the default implementation does nothing.
     * @return True if the initialization is successfull.
     */
    virtual bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * @brief Get the output of the advanceable.
     * @return a const reference of the requested object.
     */
    virtual const Output& getOutput() const = 0;

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    virtual bool setInput(const Input& input) = 0;

    /**
     * @brief Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    virtual bool isOutputValid() const = 0;

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

template <class Input, class Output> bool Advanceable<Input, Output>::close()
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
