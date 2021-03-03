/**
 * @file ModelComputationsHelper.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FLOATING_BASE_ESTIMATORS_MODEL_COMPUTATIONS_HELPER_H
#define BIPEDAL_LOCOMOTION_FLOATING_BASE_ESTIMATORS_MODEL_COMPUTATIONS_HELPER_H

// std
#include <memory>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
 * KinDynComputationsDescriptor wraps a pointer iDynTree KinDynComputations object.
 */
struct KinDynComputationsDescriptor
{
    std::shared_ptr<iDynTree::KinDynComputations> kindyn{nullptr}; /**< Pointer associated to the KinDynComputations. */

    /**
     * Constructor.
     */
    KinDynComputationsDescriptor(std::shared_ptr<iDynTree::KinDynComputations> kindyn);

    /**
     * Constructor.
     */
    KinDynComputationsDescriptor();

    /**
     * Check if the KinDynComputationsDescriptor contains a valid pointer.
     * @return True if valid, false otherwise.
     */
    bool isValid() const;
};

/**
 * Helper function that can be used to build a KinDynComputations object loaded with specified model.
 * @param handler pointer to a parameter handler interface.
 * @note the following parameters are required by the function
 * |      Parameter Name     |       Type       |                         Description                       | Mandatory |              Remark                |
 * |:-----------------------:|:----------------:|:---------------------------------------------------------:|:---------:|:----------------------------------:|
 * |      `joints_list`      | `vector<string>` |       List of the joints to be used in the reduced model  |    No     | Full model is loaded if unavailable|
 * |    `model_file_name`    |     `string`     |    file name containing the full path to the urdf model   |    Yes    |                 -                  |
 * @warning the joint list order that is obtained by loading the model without specifying `joints_list` is not stable in any way.
 *          It might not match any specific order across different environments, so one must be very careful while loading the model in this way.
 * @return A KinDynComputationsDescriptor. If one of the parameters is missing an invalid KinDynComputationsDescriptor is returned.
 */
KinDynComputationsDescriptor constructKinDynComputationsDescriptor(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);


} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FLOATING_BASE_ESTIMATORS_MODEL_COMPUTATIONS_HELPER_H
