/**
 * @file TSIDLinearTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H

#include <BipedalLocomotion/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * TSIDLinearTask specializes a LinearTask in the case of Task based Inverse Dynamics.
 */
struct TSIDLinearTask : public BipedalLocomotion::System::LinearTask
{
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H
