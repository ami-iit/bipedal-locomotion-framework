/**
 * @file JointType.h
 * @authors Ines Sorrentino
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_JOINT_TYPES_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_JOINT_TYPES_H

namespace BipedalLocomotion::RobotInterface
{
    enum class JointType
    {
        REVOLUTE, /**< Revolute joint */
        PRISMATIC /**< Prismatic joint */
    };
}

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_JOINT_TYPES_H
