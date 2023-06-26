/**
 * @file InputPort.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_INPUT_PORT_H
#define BIPEDAL_LOCOMOTION_SYSTEM_INPUT_PORT_H

namespace BipedalLocomotion
{
namespace System
{
template <class _Input> class InputPort;
} // namespace System
} // namespace BipedalLocomotion

/**
 * Basic class that represents an input port. The interface contains method to set inputs.
 */
template <class _Input> class BipedalLocomotion::System::InputPort
{
public:
    using Input = _Input;

    InputPort() = default;
    InputPort(const InputPort&) = delete;
    InputPort(InputPort&&) = delete;
    InputPort& operator=(const InputPort&) = delete;
    InputPort& operator=(InputPort&&) = delete;
    virtual ~InputPort() = default;

    /**
     * @brief Set the input of the port
     * @param input the input of the port
     * @return True in case of success and false otherwise
     */
    virtual bool setInput(const Input& input) = 0;
};

#endif // BIPEDAL_LOCOMOTION_SYSTEM_INPUT_PORT_H
