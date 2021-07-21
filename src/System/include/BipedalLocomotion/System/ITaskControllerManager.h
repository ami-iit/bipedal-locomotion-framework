/**
 * @file ITaskControllerManager.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_I_TASK_CONTROLLER_MANAGER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_I_TASK_CONTROLLER_MANAGER_H

namespace BipedalLocomotion
{
namespace System
{

/**
 * ITaskControllerManager is an interface that can help you to handle tasks containing controllers.
 * Please inherit it you want to disable or enable controllers embedded in a LinearTask
 */
struct ITaskControllerManager
{
    /**
     * Mode representing the status of the task controller.
     */
    enum class Mode
    {
        Enable,
        Disable
    };

    /**
     * Set the task control mode.
     * @param state state of the controller
     */
    virtual void setTaskControllerMode(Mode mode) = 0;

    /**
     * Get the task control mode.
     * @return the state of the controller
     */
    virtual Mode getTaskControllerMode() const = 0;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_I_TASK_CONTROLLER_MANAGER_H
