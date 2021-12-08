/**
 * @file Sink.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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

    virtual ~Sink() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_SINK_H
