/**
 * @file UnicycleUtilities.h
 * @authors Lorenzo Moretti,Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <UnicyclePlanner.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>

namespace BipedalLocomotion::Planners::UnicycleUtilities
{
/**
 * It populates the contact list.
 * @param initTime the initial time of the trajectory.
 * @param dt the time step.
 * @param inContact a vector containing the contact status.
 * @param steps a deque containing the steps.
 * @param contactFrameIndex the index of the contact frame.
 * @param contactName the name of the contact.
 * @param contactList the contact list to be populated. It should be empty or contain only the first
 *                    step (i.e., the current one already in contact at time initTime).
 */
bool getContactList(const std::chrono::nanoseconds& initTime,
                    const std::chrono::nanoseconds& dt,
                    const std::vector<bool>& inContact,
                    const std::deque<Step>& steps,
                    const int& contactFrameIndex,
                    const std::string& contactName,
                    BipedalLocomotion::Contacts::ContactList& contactList);

/**
 * Merge two steps sequences, and stores the merged output in the second one.
 * The new steps which has an impact time lower than the current time are discarded.
 * @param newSteps the new list of steps.
 * @param currentSteps the cuurent list of steps.
 * @param currentTime the current time.
 */
void mergeSteps(const std::deque<Step>& newSteps,
                std::deque<Step>& currentSteps,
                const std::chrono::nanoseconds& currentTime);

/**
 * It appends a vector to a deque.
 * @param input the input vector.
 * @param output the output deque.
 * @param initPoint the initial point where the vector has to be appended.
 * @return true if the operation is successful, false otherwise.
 */
template <typename T>
bool appendVectorToDeque(const std::vector<T>& input,
                         std::deque<T>& output,
                         const size_t& initPoint)
{
    if (initPoint > output.size())
    {
        BipedalLocomotion::log()->error("[Utilities::appendVectorToDeque] The init point has to "
                                        "be less or equal to the size of the output deque.");
        return false;
    }

    // resize the deque
    output.resize(input.size() + initPoint);

    // Advances the iterator it by initPoint positions
    typename std::deque<T>::iterator it = output.begin();
    std::advance(it, initPoint);

    // copy the vector into the deque from the initPoint position
    std::copy(input.begin(), input.end(), it);

    return true;
}

/**
 * It populates a vector from a deque.
 * @param deque the input deque.
 * @param vector the output vector.
 */
template <typename T>
void populateVectorFromDeque(const std::deque<T>& deque, std::vector<T>& vector)
{

    vector.clear();

    vector.insert(vector.end(), deque.begin(), deque.end());
}

namespace Conversions
{

void convertToBLF(const iDynTree::Transform& input, manif::SE3d& output);

void convertToBLF(const iDynTree::Twist& input, manif::SE3d::Tangent& output);

void convertToBLF(const iDynTree::Twist& input, manif::SE3d::Tangent& output);

void convertToBLF(const iDynTree::VectorDynSize& input, Eigen::VectorXd& output);

void convertToBLF(const iDynTree::Vector2& input, Eigen::Vector2d& output);

void convertToBLF(const iDynTree::Vector3& input, Eigen::Vector3d& output);

template <typename From, typename To>
void convertVector(const std::vector<From>& inputVect, std::vector<To>& outputVect)
{

    outputVect.resize(inputVect.size());

    for (size_t i = 0; i < inputVect.size(); ++i)
    {
        convertToBLF(inputVect[i], outputVect[i]);
    }
}

} // namespace Conversions
} // namespace BipedalLocomotion::Planners::UnicycleUtilities

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H
