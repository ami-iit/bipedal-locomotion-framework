/**
 * @file Helper.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_YARP_UTILITIES_HELPER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_YARP_UTILITIES_HELPER_H

// std
#include <deque>
#include <vector>

// YARP
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/Core/VectorDynSize.h>


namespace BipedalLocomotionControllers
{

/**
 * Helper for YARP library.
 */
namespace YarpUtilities
{

/**
 * Convert a value in a element of type T
 * @param value the value that will be converted
 * @tparam T return type
 * @return an element of type T
 */
template <typename T> T convertValue(const yarp::os::Value& value);

/**
 * Add a vector of string to a property of a given name.
 * @param prop yarp property;
 * @param key is the key;
 * @param list is the vector of strings that will be added into the property.
 * @return true/false in case of success/failure
 */
bool addVectorOfStringToProperty(yarp::os::Property& prop,
                                 const std::string& key,
                                 const std::vector<std::string>& list);

/**
 * Extract a double from a searchable object.
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param number is the double.
 * @return true/false in case of success/failure
 */
template <typename T>
bool getElementFromSearchable(const yarp::os::Searchable& config,
                              const std::string& key,
                              T& number);

/**
 * Extract a vector from searchable
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param vector a vector.
 * @return true/false in case of success/failure
 */
template <typename T>
bool getVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key, T& vector);

/**
 * Extract an std::vector<bool> from searchable. The specialization is required because \code{.cpp}
 * vector.data() \endcode is not defined when vector is an \code{.cpp} std::vector<bool> \endcode
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param vector a std::vector<bool>
 * @return true/false in case of success/failure
 */
template <>
bool getVectorFromSearchable<std::vector<bool>>(const yarp::os::Searchable& config,
                                                const std::string& key,
                                                std::vector<bool>& vector);

/**
 * Merge two vectors. vector = [vector, t]
 * @param vector the original vector. The new elements will be add at the end of this vector;
 * @param t vector containing the elements that will be merged with the original vector.
 * @tparam T type of the vector
 */
template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const T& t);

/**
 * Merge two vectors. vector = [vector, t]
 * @param vector the original vector. The new elements will be add at the end of this vector;
 * @param t std::vector containing the elements that will be merged with the original vector.
 * @tparam T type contained in the std::vector
 */
template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const std::vector<T>& t);

/**
 * Variadic function used to merge several vectors.
 * @param vector the original vector. The new elements will be add at the end of this vector;
 * @param t vector containing the elements that will be merged with the original vector.
 * @param args list containing all the vector that will be merged.
 */
template <typename T, typename... Args>
void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args);

/**
 * Send a variadic vector through a yarp buffered port
 * @param port is a Yarp buffered port
 * @param args list containing all the vector that will be send.
 */
template <typename... Args>
void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args);

/**
 * Add strings to a bottle.
 * @param bottle this bottle will be filled.
 * @param strings list containing all the string.
 */
void populateBottleWithStrings(yarp::os::Bottle& bottle,
                               const std::initializer_list<std::string>& strings);

} // namespace YarpUtilities
} // namespace BipedalLocomotionControllers

#include "Helper.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_YARP_UTILITIES_HELPER_H
