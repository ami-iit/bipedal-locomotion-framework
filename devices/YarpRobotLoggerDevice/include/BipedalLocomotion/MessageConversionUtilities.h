/**
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H

#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadata.h>

#include <trintrin/msgs/HumanState.h>
#include <trintrin/msgs/WearableTargets.h>
#include <trintrin/msgs/WearableData.h>

#include <string>

namespace BipedalLocomotion
{

static const std::string treeDelim = "::";

void extractMetadata(const trintrin::msgs::HumanState& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);
void extractMetadata(const trintrin::msgs::WearableTargets& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);
void extractMetadata(const trintrin::msgs::WearableData& message,
                     const std::string& prefix,
                     BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata);

void convertToVectorsCollection(const trintrin::msgs::HumanState& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);
void convertToVectorsCollection(const trintrin::msgs::WearableTargets& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);
void convertToVectorsCollection(const trintrin::msgs::WearableData& message,
                                const std::string& prefix,
                                BipedalLocomotion::YarpUtilities::VectorsCollection& collection);

}

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_MESSAGE_CONVERSION_UTILITIES_H
