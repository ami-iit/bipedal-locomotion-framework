/**
 * @file CartesianElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CARTESIAN_ELEMENT_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CARTESIAN_ELEMENT_H

#include <iDynTree/Core/VectorFixSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * CartesianElement describes a Cartesian element that will be embedded as
 * equality constraint or cost function.
 * The Cartesian element in useful to control the position/orientation/pose of a frame/CoM of the
 * robot
 */
class CartesianElement : public ControlTask
{
public:
    /** Cartesian Element Type */
    enum class Type
    {
        POSE,
        POSITION,
        ORIENTATION,
        ONE_DIMENSION
    };

    /** Name of the axis controlled in case of ONE_DIMENSION element is chosen */
    enum class AxisName
    {
        X,
        Y,
        Z
    };

private:
    /** Base acceleration index */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    /** Joint acceleration index */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    /** Control type index */
    iDynTree::IndexRange m_typeIndex{iDynTree::IndexRange::InvalidRange()};

    std::unique_ptr<LinearPD<iDynTree::Vector3>> m_positionPD; /**< 3D Position PD. */
    std::unique_ptr<OrientationPD> m_orientationPD; /**< Orientation PD. */
    std::unique_ptr<LinearPD<double>> m_oneDegreePD; /**< One Degree of freedom PD. */

    iDynTree::FrameIndex m_frameIndex; /**< Index of the frame. */

    iDynTree::MatrixDynSize m_jacobian; /**< Robotic jacobian of the element. */

    Type m_type; /**< Type of the control. */
    bool m_isInContact; /**< True if the frame if the link associated to the frame is in contact
                           with the environment. */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     * @param handler variable handler object
     * @param type type of control (position/orientation/pose)
     * @param frameName name of the frame that has to be controlled (if CoM the CoM will be
     * controlled)
     * @param axisName X, Y or Z coordinate that will be controlled
     * @throw std::runtime_error in case of an undefined variable/frame
     */
    CartesianElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                     const VariableHandler& handler,
                     const Type& type,
                     const std::string& frameName,
                     const AxisName& axisName = AxisName::X);

    /**
     * Set the desired trajectory
     * @param acceleration desired linear acceleration
     * @param velocity desired linear velocity
     * @param position desired position
     * @throw std::runtime_error if the Type is different from POSE and POSITION
     */
    void setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                              const iDynTree::Vector3& velocity,
                              const iDynTree::Vector3& position);

    /**
     * Set the desired trajectory
     * @param acceleration  desired angular acceleration
     * @param velocity desired  angular velocity
     * @param rotation desired rotation matrix
     * @throw std::runtime_error if the Type is different from POSE and ORIENTATION
     */
    void setDesiredTrajectory(const iDynTree::Vector3& acceleration,
                              const iDynTree::Vector3& velocity,
                              const iDynTree::Rotation& rotation);

    /**
     * Set the desired trajectory
     * @param acceleration desired spatial acceleration
     * @param velocity angular desired spatial velocity
     * @param transform desired homogeneous transform
     * @throw std::runtime_error if the Type is different from POSE
     */
    void setDesiredTrajectory(const iDynTree::SpatialAcc& acceleration,
                              const iDynTree::Twist& velocity,
                              const iDynTree::Transform& transform);

    /**
     * Set the desired trajectory
     * @param acceleration desired acceleration
     * @param velocity angular desired velocity
     * @param transform desired position
     * @throw std::runtime_error if the Type is different from ONE_DIMENSION
     */
    void setDesiredTrajectory(const double& acceleration,
                              const double& velocity,
                              const double& position);

    /**
     * Set the linear PD gains
     * @param kp proportional gain
     * @param kd derivative gain
     * @throw std::runtime_error if the Type is different from POSE and POSITION
     */
    void setLinearPDGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd);

    /**
     * Set the orientation PD gains
     * @param c0 c0 gain
     * @param c1 derivative gain
     * @param c2 proportional gain
     * @throw std::runtime_error if the Type is different from ORIENTATION and POSITION
     */
    void setOrientationPDGains(const double& c0, const double& c1, const double& c2);

    /**
     * Set the linear PD gains
     * @param kp proportional gain
     * @param kd derivative gain
     * @throw std::runtime_error if the Type is different from ONE_DIMENSION
     */
    void setOneDegreePDGains(const double& kp, const double& kd);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() final;

    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
