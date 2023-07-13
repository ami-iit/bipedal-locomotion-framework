/**
 * @file IClock.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ICLOCK_H
#define BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ICLOCK_H

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>

#include <BipedalLocomotion/System/IClock.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

template <class IClockBase = BipedalLocomotion::System::IClock>
class IClockTrampoline : public IClockBase
{
public:
    using IClockBase::IClockBase;

    std::chrono::nanoseconds now() override
    {
        PYBIND11_OVERLOAD_PURE(std::chrono::nanoseconds, IClockBase, now);
    }

    void sleepFor(const std::chrono::nanoseconds& sleepDuration) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IClockBase, "sleep_for", sleepFor);
    }

    void sleepUntil(const std::chrono::nanoseconds& time) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, IClockBase, "sleep_until", seepUntil);
    }

    void yield() override
    {
        PYBIND11_OVERLOAD_PURE(void, IClockBase, yeld);
    }
};

template <class ClockFactoryBase = BipedalLocomotion::System::ClockFactory>
class ClockFactoryTrampoline : public ClockFactoryBase
{
public:
    using ClockFactoryBase::ClockFactoryBase;

    ::BipedalLocomotion::System::IClock& createClock() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(::BipedalLocomotion::System::IClock&,
                                    ClockFactoryBase,
                                    "create_clock",
                                    createClock);
    }
};

void CreateIClock(pybind11::module& module);
void CreateClockFactory(pybind11::module& module);

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ICLOCK_H
