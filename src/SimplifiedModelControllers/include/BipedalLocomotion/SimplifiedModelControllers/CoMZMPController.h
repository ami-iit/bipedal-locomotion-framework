/**
 * @file CoMZMPController.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <manif/SO2.h>

namespace BipedalLocomotion
{
namespace SimplifiedModelControllers
{

struct CoMZMPControllerInput
{
    Eigen::Vector2d desiredCoMVelocity; /**< Desired CoM velocity */
    Eigen::Vector2d desiredCoMPosition; /**< Desired CoM position */
    Eigen::Vector2d desiredZMPPosition; /**< Desired ZMP position */

    Eigen::Vector2d CoMPosition; /**< CoM position */
    Eigen::Vector2d ZMPPosition; /**< ZMP position */

    /** the yaw angle represents the I_R_B rotation matrix. The rotation brings a vector expressed
     * in the frame rigidly attached to the CoM (B) to the inertial frame (I). The yaw angle allows
     * the user to have different gains on the forward and lateral walking direction. */
    double angle;
};

using CoMZMPControllerOutput = Eigen::Vector2d;

/**
 * CoMZMPController implements the following control law
 * \f[
 * \dot{x} = \dot{x}^* - K _ {zmp} (r^*_{zmp} - r_{zmp}) + K _ {com} (x^* - x)
 * \f]
 * where \f$x\f$ is the CoM position and \f$r_{zmp}\f$ the ZMP position.
 * The gains \f$K_{zmp}\f$ and \f$K_{com}\f$ depends on the orientation of a frame rigidly attached
 * to the CoM (B) w.r.t. the inertial frame (I).
 * The rotation matrix \f${}^I R _ B\f$ depends only on the yaw angle.
 * <img
 * src="https://user-images.githubusercontent.com/16744101/128683430-f63c3c34-404a-42d9-b864-2437445abe38.png"
 * alt="CoMZMPController">
 * @note Since the controller is based on the assumption of the Linear Inverted Pendulum Model
 * (LIPM), the CoM height velocity is always equal to zero.
 * @note The yaw angle allows the user to have different gains on the forward and
 * lateral walking direction.
 * @note The design of the controller is taken from [Y. Choi, D. Kim, Y. Oh, and B.-j. J. You, _On
 * the Walking Control for Humanoid Robot Based on Kinematic Resolution of CoM Jacobian With
 * Embedded Motion_](https://doi.org/10.1109/ROBOT.2006.1642102).
 */
class CoMZMPController : public System::Advanceable<CoMZMPControllerInput, CoMZMPControllerOutput>
{
public:
    /**
     * Initialize the controller.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |  Parameter Name  |        Type      |                                         Description                                                   | Mandatory |
     * |:----------------:|:----------------:|:-----------------------------------------------------------------------------------------------------:|:---------:|
     * |    `com_gain`    | `vector<double>` | 2D-vector containing the gains of the CoM written in a frame rigidly attached to the simplified model |    Yes    |
     * |    `zmp_gain`    | `vector<double>` | 2D-vector containing the gains of the ZMP written in a frame rigidly attached to the simplified model |    Yes    |
     * @return true in case of success/false otherwise.
     */
     bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

     /**
      * Get the the controller output.
      * @return The velocity of the CoM.
      */
     const Output& getOutput() const final;

     /**
      * Determines the validity of the object retrieved with getOutput()
      * @return True if the object is valid, false otherwise.
      */
     bool isOutputValid() const final;

     /**
      * Compute the control law.
      * @return True if the advance is successfull.
      */
     bool advance() final;

     /**
     * Set the input of the advanceable block.
     * @param input the CoMZMPControllerInput struct
     * @return true in case of success and false otherwise.
     */
     bool setInput(const Input& input) final;

     /**
      * Set the desired set-point.
      * @param CoMVelocity a 2d-vector containing the x and y coordinate of the CoM velocity.
      * @param CoMPosition a 2d-vector containing the x and y coordinate of the CoM position.
      * @param ZMPPosition a 2d-vector containing the x and y coordinate of the ZMP position.
      */
     void setSetPoint(Eigen::Ref<const Eigen::Vector2d> CoMVelocity,
                      Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                      Eigen::Ref<const Eigen::Vector2d> ZMPPosition);

     /**
      * Set the state feedback
      * @param CoMPosition a 2d-vector containing the x and y coordinate of the CoM position.
      * @param ZMPPosition a 2d-vector containing the x and y coordinate of the ZMP position.
      * @param I_R_B rotation matrix that brings a vector expressed in the frame rigidly attached to
      * the CoM (B) to the inertial frame (I).
      * @note Since the controller is based on the LIPM assumption, the rotation matrix contains
      * only a rotation along the z axis. The yaw angle allows the user to have different gains on
      * the forward and lateral walking direction.
      */
     void setFeedback(Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                      Eigen::Ref<const Eigen::Vector2d> ZMPPosition,
                      const manif::SO2d& I_R_B);

     /**
      * Set the state feedback
      * @param CoMPosition a 2d-vector containing the x and y coordinate of the CoM position.
      * @param ZMPPosition a 2d-vector containing the x and y coordinate of the ZMP position.
      * @param angle the yaw angle (in radians) represents the \f${}^I R_B\f$ rotation matrix. The
      * rotation brings a vector expressed in the frame rigidly attached to the CoM (B) to the
      * inertial frame (I). The yaw angle allows the user to have different gains on the forward and
      * lateral walking direction.
      */
     void setFeedback(Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                      Eigen::Ref<const Eigen::Vector2d> ZMPPosition,
                      const double angle);

 private:
     manif::SO2d m_I_R_B{manif::SO2d::Identity()};
     Eigen::Vector2d m_CoMGain{Eigen::Vector2d::Zero()};
     Eigen::Vector2d m_ZMPGain{Eigen::Vector2d::Zero()};

     Eigen::Vector2d m_controllerOutput{Eigen::Vector2d::Zero()};

     Eigen::Vector2d m_desiredCoMVelocity{Eigen::Vector2d::Zero()};
     Eigen::Vector2d m_CoMPosition{Eigen::Vector2d::Zero()};
     Eigen::Vector2d m_desiredCoMPosition{Eigen::Vector2d::Zero()};
     Eigen::Vector2d m_ZMPPosition{Eigen::Vector2d::Zero()};
     Eigen::Vector2d m_desiredZMPPosition{Eigen::Vector2d::Zero()};

     bool m_isOutputValid{false};
     bool m_isInitalized{false};
};

} // namespace SimplifiedModelControllers
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H
