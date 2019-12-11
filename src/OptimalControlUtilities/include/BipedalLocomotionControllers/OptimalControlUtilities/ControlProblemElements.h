/**
 * @file ControlProblemElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H

#include <memory>
#include <unordered_map>
#include <vector>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/PID.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

#include <exception>

namespace BipedalLocomotionControllers
{
/**
 * The struct frame defines a Frame in a ControlProblemElement
 */
typedef struct
{
    iDynTree::FrameIndex indexInModel; /**< Index of the frame in the Model */

    /** Index and size of the element in the ControlProblemElement */
    iDynTree::IndexRange indexRangeInElement{iDynTree::IndexRange::InvalidRange()};
} Frame;

/**
 * ControlProblemElement describes a general control problem element. The
 * element is considered linear w.r.t the unknown variable.
 * i.e. \f$ A x \f$ where \f$ A \f$ is the element matrix and \f$ x \f$ the
 * unknown variable
 */
class ControlProblemElement
{
protected:
    iDynTree::KinDynComputations* m_kinDyn; /**< KinDyn pointer object */
    iDynTree::MatrixDynSize m_A; /**< Element Matrix */
    std::string m_name; /**< Name of the element */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    ControlProblemElement(iDynTree::KinDynComputations& kinDyn);

    /**
     * Get the size of the element (i.e. the number of rows of the element matrix)
     * @return the size of the element
     */
    int getSize() const;

    /**
     * Get the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() = 0;

    /**
     * Get the name of the element
     * @return the description of the element
     */
    const std::string& getName() const;
};

/**
 * CostFunctionOrEqualityConstraintElement describes a control problem
 * element that will be embedded as cost function or as equality constraint.
 * The element is described by \f$ A \f$ and \f$ b \f$. \f$ A \f$ is the
 * element matrix and \f$ x \f$ and \f$ b \f$ the element vector.
 * In case of cost function \f$ A \f$ and \f$ b \f$ represents:
 * \f$ \norm{Ax - b}^2 \f$. In case of equality constraint
 * \f$ Ax = b \f$
 */
class CostFunctionOrEqualityConstraintElement : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_b; /**< Element Vector */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    CostFunctionOrEqualityConstraintElement(iDynTree::KinDynComputations& kinDyn);

    /**
     * Get the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() = 0;
};

/**
 * InequalityConstraintElement describes a control problem
 * element that will be embedded as inequality constraint.
 * The element is described by \f$ A \f$, \f$ l \f$ and  \f$ u \f$.
 * \f$ l \le Ax \le u \f$
 */
class InequalityConstraintElement : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_l; /**< Lower bound */
    iDynTree::VectorDynSize m_u; /**< Upper bound */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    InequalityConstraintElement(iDynTree::KinDynComputations& kinDyn);

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() = 0;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() = 0;
};

/**
 * CartesianElement describes a Cartesian element that will be embedded as
 * equality constraint or cost function.
 * The Cartesian element in useful to control the position/orientation/pose of a frame/CoM of the
 * robot
 */
class CartesianElement : public CostFunctionOrEqualityConstraintElement
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

    std::unique_ptr<PositionPID> m_positionPID; /**< 3D Position PID. */
    std::unique_ptr<OrientationPID> m_orientationPID; /**< Orientation PID. */
    std::unique_ptr<OneDegreePID> m_oneDegreePID; /**< One Degree of freedom PID. */

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
    CartesianElement(iDynTree::KinDynComputations& kinDyn,
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
     * Set the linear PID gains
     * @param kp proportional gain
     * @param kd derivative gain
     * @throw std::runtime_error if the Type is different from POSE and POSITION
     */
    void setLinearPIDGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd);

    /**
     * Set the orientation PID gains
     * @param c0 c0 gain
     * @param c1 derivative gain
     * @param c2 proportional gain
     * @throw std::runtime_error if the Type is different from ORIENTATION and POSITION
     */
    void setOrientationPIDGains(const double& c0, const double& c1, const double& c2);

    /**
     * Set the linear PID gains
     * @param kp proportional gain
     * @param kd derivative gain
     * @throw std::runtime_error if the Type is different from ONE_DIMENSION
     */
    void setOneDegreePIDGains(const double& kp, const double& kd);

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

/**
 * SystemDynamicsElement describes the System Dynamics that will be embedded as
 * equality constraint or cost function.
 */
class SystemDynamicsElement : public CostFunctionOrEqualityConstraintElement
{
    /** Index range of the base acceleration */
    iDynTree::IndexRange m_baseAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    /** Index range of the joint acceleration */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};

    /** Index range of the joint torque */
    iDynTree::IndexRange m_jointTorqueIndex{iDynTree::IndexRange::InvalidRange()};

    iDynTree::MatrixDynSize m_massMatrix; /**< Floating-base mass matrix  */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Coriolis and
                                                                         Gravitational term  */
    iDynTree::MatrixDynSize m_jacobianMatrix; /**< Jacobian Matrix  */

    std::vector<Frame> m_framesInContact; /**< Vectors containing the frames in contact with the
                                             environment */

    iDynTree::MatrixDynSize m_reflectedInertia; /**< Reflected inertia matrix  */
    bool m_useReflectedInertia; /**< If true the reflected inertia will be used  */

public:
    /**
     * Constructor. If you call this constructor the motor reflected inertia is disabled
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @throw std::runtime_error if the frame is not defined
     */
    SystemDynamicsElement(iDynTree::KinDynComputations& kinDyn,
                          const VariableHandler& handler,
                          const std::vector<std::pair<std::string, std::string>>& framesInContact);

    /**
     * Constructor. If you call this constructor the motor reflected inertia is enable
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @param gamma diagonal matrix containing the gear box ratios;
     * @param motorsInertia vector containing the inertia of the motors
     * @param harmonicDriveInertia vector containing the inertia of the harmonic drive
     * @param r torso parameter http://wiki.icub.org/wiki/ICub_coupled_joints
     * @param R torso parameter http://wiki.icub.org/wiki/ICub_coupled_joints
     * @param t shoulder parameter http://wiki.icub.org/wiki/ICub_coupled_joints
     * @throw std::runtime_error if the frame is not defined
     * @note this is iCub specific. In the next version it will be removed
     */
    [[deprecated(
        "This function is iCub specific. In the next implementation will be removed. If "
        "you want to include regularization element for the mass matrix please call "
        "SystemDynamicsElement(iDynTree::KinDynComputations&, const VariableHandler&, "
        "const std::vector<std::pair<std::string, std::string>>&, const "
        "iDyntree::MatrixDynsize&)")]]
    SystemDynamicsElement(iDynTree::KinDynComputations& kinDyn,
                          const VariableHandler& handler,
                          const std::vector<std::pair<std::string, std::string>>& framesInContact,
                          const iDynTree::VectorDynSize& gamma,
                          const iDynTree::VectorDynSize& motorsInertia,
                          const iDynTree::VectorDynSize& harmonicDriveInertia,
                          const double& r,
                          const double& R,
                          const double& t);

    /**
     * Constructor. If you call this constructor the motor reflected inertia is enable
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @param regularizationMatrix joints regularization mass matrix. It can be used to handle
     * the motor reflected inertia
     * @throw std::runtime_error if the frame is not defined or the regularizationMatrix size is
     * no coherent with the number of joints
     */
    SystemDynamicsElement(iDynTree::KinDynComputations& kinDyn,
                          const VariableHandler& handler,
                          const std::vector<std::pair<std::string, std::string>>& framesInContact,
                          const iDynTree::MatrixDynSize& regularizationMatrix);

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
};

/**
 * CentroidalLinearMomentumElement describes the Centroidal linear momentum
 */
class CentroidalLinearMomentumElement : public CostFunctionOrEqualityConstraintElement
{
private:
    iDynTree::Vector3 m_VRP; /**< Position of the Virtual Repellent Point */
    double m_robotMass; /**< Mass of the robot */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * the name of the frame in the variableHandler
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalLinearMomentumElement(iDynTree::KinDynComputations& kinDyn,
                                    const VariableHandler& handler,
                                    const std::vector<std::string>& framesInContact);

    /**
     * Set the desired VRP
     * @param VRP position of the virtual repellent point
     */
    void setVRP(const iDynTree::Vector3& VRP);

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
};

/**
 * CentroidalAngularMomentumElement describes the Centroidal angular momentum
 */
class CentroidalAngularMomentumElement : public CostFunctionOrEqualityConstraintElement
{
private:
    std::unique_ptr<PositionPID> m_pid; /**< Linear PID */
    std::vector<Frame> m_framesInContact; /**< Vectors containing the frames in contact with the
                                             environment */
    iDynTree::Vector3 m_zero; /**< Vector of zero elements */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @throw std::runtime_error if the frame is not defined
     */
    CentroidalAngularMomentumElement(
        iDynTree::KinDynComputations& kinDyn,
        const VariableHandler& handler,
        const std::vector<std::pair<std::string, std::string>>& framesInContact);

    /**
     * Set the desired centroidal angular momentum
     * @param centroidalAngularMomentumVelocity desired angular momentum rate of change
     * @param centroidalAngularMomentum desired angular momentum
     */
    void
    setDesiredCentroidalAngularMomentum(const iDynTree::Vector3& centroidalAngularMomentumVelocity,
                                        const iDynTree::Vector3& centroidalAngularMomentum);

    /**
     * Set the controller gain
     * @param kp controller gain
     */
    void setGain(const double& kp);

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
};

/**
 * RegularizationElement describes an element used to regularize unknown variables
 * It is in general used as CostFunctionElement of the form  \f$ x^\top A^\top A x \f$
 */
class RegularizationElement : public CostFunctionOrEqualityConstraintElement
{
public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param variableName name of the unknown variable that should minimized
     * @throw std::runtime_error if the variableName is not defined in the handler
     */
    RegularizationElement(iDynTree::KinDynComputations& kinDyn,
                          const VariableHandler& handler,
                          const std::string& variableName);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() override;
};

/**
 * RegularizationWithControlElement describes an element used to regularize unknown variables using
 * a control law It is in general used as CostFunctionElement of the form  \f$ x^\top A^\top A x +
 * g^\top x \f$
 */
class RegularizationWithControlElement : public RegularizationElement
{
    iDynTree::VectorDynSize m_kp; /**< Proportional gain */
    iDynTree::VectorDynSize m_kd; /**< Derivative gain */

    iDynTree::VectorDynSize m_desiredPosition; /**< Desired position. */
    iDynTree::VectorDynSize m_desiredVelocity; /**< Desired velocity. */
    iDynTree::VectorDynSize m_desiredAcceleration; /**< Desired acceleration. */

    iDynTree::VectorDynSize m_position; /**< Current position. */
    iDynTree::VectorDynSize m_velocity; /**< Current velocity. */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param variableName name of the unknown variable that should minimized
     * @throw std::runtime_error if the variableName is not defined in the handler
     */
    RegularizationWithControlElement(iDynTree::KinDynComputations& kinDyn,
                                     const VariableHandler& handler,
                                     const std::string& variableName);

    /**
     * Set the desired trajectory
     * @param acceleration desired acceleration
     * @param velocity  desired velocity
     * @param position desired position
     */
    void setDesiredTrajectory(const iDynTree::VectorDynSize& acceleration,
                              const iDynTree::VectorDynSize& velocity,
                              const iDynTree::VectorDynSize& position);

    /**
     * Set the state
     * @param velocity current velocity
     * @param position current position
     */
    void setState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position);

    /**
     * Set the PID gains
     * @param kp proportional gain
     * @param kd derivative gain
     */
    void setPIDGains(const iDynTree::VectorDynSize& kp, const iDynTree::VectorDynSize& kd);

    /**
     * Get (and compute) the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB() final;
};

/**
 * ZMPElement handles the tracking of the global Zero Moment Point (ZMP)
 */
class ZMPElement : public CostFunctionOrEqualityConstraintElement
{
    std::vector<Frame> m_framesInContact; /**< Vectors containing the frames in contact with the
                                             environment */
    iDynTree::Vector2 m_ZMP; /**< Desired global Zero Moment Point position */
    iDynTree::Position m_contactFramePosition; /**< Position of the contact frame */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param framesInContact vector containing the frames in contact. Each element of the vector is
     * a pair containing the name of the frame in the variableHandler and in the model.
     * @throw std::runtime_error if the frame is not defined
     */
    ZMPElement(iDynTree::KinDynComputations& kinDyn,
               const VariableHandler& handler,
               const std::vector<std::pair<std::string, std::string>>& framesInContact);

    /**
     * Set the desired ZMP
     * @param ZMP position of the zero moment point
     */
    void setDesiredZMP(const iDynTree::Vector2& ZMP);

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
};

/**
 * ContactWrenchFeasibilityElement handles the constraints related to the contact wrenches
 * such that: Unilateral constraint, friction cone, torsional friction parameter
 * and position of the local Center of Pressure
 */
class ContactWrenchFeasibilityElement : public InequalityConstraintElement
{
    Frame m_frameInContact; /**< Frame in contact with the environment */
    iDynTree::Rotation m_rotationMatrix; /**< Frame rotation matrix */
    iDynTree::MatrixDynSize m_AInBodyFrame; /**< Constrain matrix written in body frame */
    double m_infinity; /**< Double representing the infinity */
    double m_minimalNormalForce; /**< Minimal normal force required */
    unsigned int m_nominalForceConstraintIndex; /**< Index of the minimal normal force required */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param frameInContact pair containing the frames in contact. The first
     * element is the name of the frame in the handler while the second is
     * the name in the model
     * @param numberOfPoints number of points used to approximate a quarter of the friction cone
     * @param staticFrictionCoefficient static friction coefficient
     * @param torsionalFrictionCoefficient torsional friction coefficient
     * @param minimalNormalForce minimal normal force required
     * @param footLimitX foot size on the X axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param footLimitY foot size on the Y axis written with respect the foot frame. The
     * first value should be negative while the second one positive
     * @param infinity double representing the infinity
     * @throw std::runtime_error if the frame is not defined
     */
    ContactWrenchFeasibilityElement(iDynTree::KinDynComputations& kinDyn,
                                    const VariableHandler& handler,
                                    const std::pair<std::string, std::string>& frameInContact,
                                    const int& numberOfPoints,
                                    const double& staticFrictionCoefficient,
                                    const double& torsionalFrictionCoefficient,
                                    const double& minimalNormalForce,
                                    const iDynTree::Vector2& footLimitX,
                                    const iDynTree::Vector2& footLimitY,
                                    const double& infinity);

    /**
     * Set if the link is in contact with the environment
     * @param isInContact true if the link associated to the frame is in contact with the
     * environment
     */
    void isInContact(bool isInContact);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() final;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() final;
};

/**
 * JointValuesFeasibilityElement handles the robot joint limits
 */
class JointValuesFeasibilityElement : public InequalityConstraintElement
{
    /** Index of the joint acceleration */
    iDynTree::IndexRange m_jointAccelerationIndex{iDynTree::IndexRange::InvalidRange()};
    iDynTree::VectorDynSize m_jointPositions; /**< Joint position in radiant */
    iDynTree::VectorDynSize m_jointVelocities; /**< Joint velocities in radiant per second */

    iDynTree::VectorDynSize m_minJointPositionsLimit; /**< Min joint position limit in radiant */
    iDynTree::VectorDynSize m_maxJointPositionsLimit; /**< Max joint position limit in radiant */

    double m_samplingTime; /**< Sampling time */

public:
    /**
     * Constructor.
     * @param kinDyn an iDynTree kinDyn computation object
     * @param handler the variable handler object
     * @param variableName name of the variable in the handler
     * @param maxJointPositionsLimit Max joint position limit in radiant
     * @param minJointPositionsLimit Min joint position limit in radiant
     * @param samplingTime the controller rate in second
     * @throw std::runtime_error if variableName is not defined in the handler
     */
    JointValuesFeasibilityElement(iDynTree::KinDynComputations& kinDyn,
                                  const VariableHandler& handler,
                                  const std::string& variableName,
                                  const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                  const iDynTree::VectorDynSize& minJointPositionsLimit,
                                  const double& samplingTime);

    /**
     * Get (and compute) the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA() final;

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound() final;

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound() final;
};
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
