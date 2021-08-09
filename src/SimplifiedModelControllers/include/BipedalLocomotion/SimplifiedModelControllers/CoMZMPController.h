/**
 * @file CoMZMPController.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H
#define BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>
#include <manif/SO2.h>

namespace BipedalLocomotion
{
namespace SimplifiedModelControllers
{

/**
 * CoMZMPController implements the following control law
 * \f[
 * \dot{x} = \dot{x}^* - K _ {zmp} (r^*_{zmp} - r_{zmp}) + K _ {com} (x^* - x)
 * \f]
 * where \f$x\f$ is the CoM position and \f$r_{zmp}\f$ the ZMP position.
 * The gains \f$K_{zmp}\f$ and \f$K_{com}\f$ depends on the orientation of a frame rigidly attached
 * to the CoM w.r.t. the inertial frame.
 * <img src="https://user-images.githubusercontent.com/16744101/128683430-f63c3c34-404a-42d9-b864-2437445abe38.png" alt="CoMZMPController">
 * @note Since the controller is based on the assumption of the Linear Inverted Pendulum Model (LIPM),
 * the CoM height velocity is always equal to zero.
 * @note The design of the controller is taken from [Y. Choi, D. Kim, Y. Oh, and B.-j. J. You, _On the
 * Walking Control for Humanoid Robot Based on Kinematic Resolution of CoM Jacobian With Embedded
 * Motion_](https://doi.org/10.1109/ROBOT.2006.1642102).
 */
class CoMZMPController : public BipedalLocomotion::System::Source<Eigen::Vector2d>
{
public:
    /**
     * Initialize the controller.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |  Parameter Name  |        Type      |                                       Description                                                  | Mandatory |
     * |:----------------:|:----------------:|:--------------------------------------------------------------------------------------------------:|:---------:|
     * |    `com_gain`    | `vector<double>` | Vector containing the gains of the CoM written in a frame rigidly attached to the simplified model |    Yes    |
     * |    `zmp_gain`    | `vector<double>` | Vector containing the gains of the ZMP written in a frame rigidly attached to the simplified model |    Yes    |
     * @return true in case of success/false otherwise.
     */
     bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

     /**
      * Get the the controller output.
      * @return The velocity of the CoM.
      */
     const Eigen::Vector2d& getOutput() const final;

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
      * only a rotation along the z axis.
      */
     void setFeedback(Eigen::Ref<const Eigen::Vector2d> CoMPosition,
                      Eigen::Ref<const Eigen::Vector2d> ZMPPosition,
                      const manif::SO2d& I_R_B);

     /**
      * Set the state feedback
      * @param CoMPosition a 2d-vector containing the x and y coordinate of the CoM position.
      * @param ZMPPosition a 2d-vector containing the x and y coordinate of the ZMP position.
      * @param angle rotation angle in degrees representing a rotation along the z axis.
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
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SIMPLIFIED_MODEL_CONTROLLERS_COM_ZMP_CONTROLLER_H
