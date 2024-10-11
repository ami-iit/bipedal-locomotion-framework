/**
 * @file Helper.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_YARP_UTILITIES_HELPER_H
#define BIPEDAL_LOCOMOTION_YARP_UTILITIES_HELPER_H

// std
#include <deque>
#include <vector>

// YARP
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/VectorDynSize.h>

namespace BipedalLocomotion
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
 * Convert a value in a element of type int
 * @param value the value that will be converted
 * @return an integer
 */
template <> int convertValue<int>(const yarp::os::Value& value);

/**
 * Convert a value in a element of type double
 * @param value the value that will be converted
 * @return a double
 */
template <> double convertValue<double>(const yarp::os::Value& value);

/**
 * Convert a value in a element of type string
 * @param value the value that will be converted
 * @return a string
 */
template <> std::string convertValue<std::string>(const yarp::os::Value& value);

/**
 * Convert a value in a element of type Boolean
 * @param value the value that will be converted
 * @return a Boolean
 */
template <> bool convertValue<bool>(const yarp::os::Value& value);


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
 * Extract an <code>std::vector<bool></code> from searchable. The specialization is required because
 * vector.data() is not defined when vector is an <code>std::vector<bool></code>
 * @param config is the searchable object;
 * @param key the name to check for;
 * @param vector a <code>std::vector<bool></code>
 * @return true/false in case of success/failure
 */
template <>
bool getVectorFromSearchable<std::vector<bool>>(const yarp::os::Searchable& config,
                                                const std::string& key,
                                                std::vector<bool>& vector);

/**
 * Append a scalar to a vector. vector = [vector, t]
 * @param vector the original vector. The new elements will be add at the end of this vector;
 * @param t is a container or a scalar. If t is a container it has to be an iterable object (the
 * element has the methods <code>T::begin()</code> and <code>T::end()</code> or the <code>operator[]</code> has
 * to be defined
 * @warning If \at does not satisfies the previous assumptions the compilation will fail with a
 * static assertion
 */
template <typename T>
void mergeSigVector(yarp::sig::Vector& vector, const T& t);

/**
 * Variadic function used to merge several vectors.
 * @param vector the original vector. The new elements will be add at the end of this vector;
 * @param t is a container or a scalar. If t is a container it has to be an iterable object (the
 * element has the methods <code>T::begin()</code> and <code>T::end()</code>) or the operator[] has
 * to be defined
 * @param args list containing all the vector that will be merged.
 * @warning If \a t does not satisfies the previous assumptions the compilation will fail with a
 * static assertion
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
} // namespace BipedalLocomotion
#include "Helper.tpp"

#endif // BIPEDAL_LOCOMOTION_YARP_UTILITIES_HELPER_H
