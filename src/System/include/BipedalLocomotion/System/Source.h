/**
 * @file Source.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_SOURCE_H
#define BIPEDAL_LOCOMOTION_SYSTEM_SOURCE_H

#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Source is a template specialization of Advanceable and represents a block that does not contains
 * input. In this case Source::setInput() will return always true.
 */
template <class Output> class Source : public Advanceable<EmptySignal, Output>
{

public:
    bool setInput(const EmptySignal& input) final
    {
        return true;
    }

    Source() = default;
    Source(const Source&) = delete;
    Source(Source&&) = delete;
    Source& operator=(const Source&) = delete;
    Source& operator=(Source&&) = delete;
    virtual ~Source() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_SOURCE_H
