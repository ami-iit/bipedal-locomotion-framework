/**
 * @file OutputPort.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_OUTPUT_PORT_H
#define BIPEDAL_LOCOMOTION_SYSTEM_OUTPUT_PORT_H

namespace BipedalLocomotion
{
namespace System
{
template <class _Output> class OutputPort;
} // namespace System
} // namespace BipedalLocomotion

/**
 * Basic class that represents an output port. The interface contains method to set inputs and
 * output.
 */
template <class _Output> class BipedalLocomotion::System::OutputPort
{
public:
    using Output = _Output;

    OutputPort() = default;
    OutputPort(const OutputPort&) = delete;
    OutputPort(OutputPort&&) = delete;
    OutputPort& operator=(const OutputPort&) = delete;
    OutputPort& operator=(OutputPort&&) = delete;
    virtual ~OutputPort() = default;

    /**
     * @brief Get the output of the port.
     * @return a const reference of the requested object.
     */
    virtual const Output& getOutput() const = 0;

    /**
     * @brief Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    virtual bool isOutputValid() const = 0;
};

#endif // BIPEDAL_LOCOMOTION_SYSTEM_OUTPUT_PORT_H
