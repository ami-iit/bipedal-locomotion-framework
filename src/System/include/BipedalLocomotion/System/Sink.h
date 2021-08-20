/**
 * @file Sink.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_SINK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_SINK_H

#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Sink is a template specialization of Advanceable and represents a block that does not contains
 * output. In this case Sink::getOutput() will return an EmptySignal.
 */
template <class Input> class Sink : public Advanceable<Input, EmptySignal>
{
    EmptySignal m_out;

public:
    const EmptySignal& getOutput() const final
    {
        return m_out;
    }
    bool isOutputValid() const final
    {
        return true;
    }

    Sink() = default;
    Sink(const Sink&) = delete;
    Sink(Sink&&) = delete;
    Sink& operator=(const Sink&) = delete;
    Sink& operator=(Sink&&) = delete;
    virtual ~Sink() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_SINK_H
