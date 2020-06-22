/**
 * @file Advanceable.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H

namespace BipedalLocomotion
{
namespace System
{
template<typename T>
class Advanceable;
}
}

/**
 * Basic class that contains only a get() and an advance() method.
 * It is possible to check the validity of the retrieved value with isValid()
 */
template<typename T>
class BipedalLocomotion::System::Advanceable
{
public:

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    virtual const T& get() const = 0;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    virtual bool isValid() const = 0;

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successfull.
     */
    virtual bool advance() = 0;
};

#endif //BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_H
