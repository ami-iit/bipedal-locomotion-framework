/**
 * @file RobotDynamicsEstimator.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_ROBOT_DYNAMICS_ESTIMATOR_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_ROBOT_DYNAMICS_ESTIMATOR_H

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <map>

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * RobotDynamicsEstimatorInput of the RobotDynamicsEstimator containing both the inputs and the
 * measurements for the Unscented Kalman Filter
 */
struct RobotDynamicsEstimatorInput
{
    manif::SE3d basePose; /**< Pose describing the robot base position and orientation. */
    manif::SE3d::Tangent baseVelocity; /**< Velocity of the robot base. */
    manif::SE3d::Tangent baseAcceleration; /**< Mixed acceleration of the robot base. */
    Eigen::VectorXd jointPositions; /**< Joints positions in rad. */
    Eigen::VectorXd jointVelocities; /**< Joints velocities in rad per sec. */
    Eigen::VectorXd motorCurrents; /**< Motor currents in Ampere. */
    std::map<std::string, Eigen::VectorXd> ftWrenches; /**< Wrenches measured by force/torque sensors. */
    std::map<std::string, Eigen::VectorXd> linearAccelerations; /**< Linear accelerations measured by accelerometer sensors. */
    std::map<std::string, Eigen::VectorXd> angularVelocities; /**< Angular velocities measured by gyroscope sensors. */
    Eigen::VectorXd frictionTorques; /**< Friction torques in Nm. */
};

/**
 * RobotDynamicsEstimatorOutput of the RobotDynamicsEstimator which represents the estimation of the
 * Unscented Kalman Filter
 */
struct RobotDynamicsEstimatorOutput
{
    Eigen::VectorXd ds; /**< Joints velocities in rad per sec. */
    Eigen::VectorXd tau_m; /**< Motor toruqes in Nm. */
    Eigen::VectorXd tau_F; /**< Motor friction torques in Nm. */
    std::map<std::string, Eigen::VectorXd> ftWrenches; /**< Wrenches at the force/torque sensors. */
    std::map<std::string, Eigen::VectorXd> ftWrenchesBiases; /**< Biases of the force/torque sensors. */
    std::map<std::string, Eigen::VectorXd> accelerometerBiases; /**< Biases of the accelerometer sensors. */
    std::map<std::string, Eigen::VectorXd> gyroscopeBiases; /**< Biases of the gyroscope sensors. */
    std::map<std::string, Eigen::VectorXd> contactWrenches; /**< External contact wrenches. */
    std::map<std::string, Eigen::VectorXd> linearAccelerations; /**< Linear acceleration of the accelerometer frames. */
    std::map<std::string, Eigen::VectorXd> angularVelocities; /**< Angular velocity of the gyroscope frames. */
};

/**
 * RobotDynamicsEstimator is a concrete class and implements a robot dynamics estimator.
 * The estimator is here implemented as an Unscented Kalman Filter. The user can build the estimator
 * by using the build method or can initialize the RobotDynamicsEstimator object and then create the
 * UkfState model and UkfMeasurement model which are then passed to the `bfl::UKFPrediction` and
 * `bfl::UKFCorrection` objects respectively. To run an estimation step the user should set the
 * input using `setInput`, call the `advance` method to perform an estimation step, use `getOutput`
 * to get the estimation result.
 */
class RobotDynamicsEstimator : public System::Advanceable<RobotDynamicsEstimatorInput, RobotDynamicsEstimatorOutput>
{
    /**
     * Private implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * Constructor.
     */
    RobotDynamicsEstimator();

    /**
     * Destructor.
     */
    virtual ~RobotDynamicsEstimator();

    /**
     * Initialize the RobotDynamicsEstimator.
     * @param handler pointer to the IParametersHandler
     * @note
     * |   Group   |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * | `GENERAL` |       `sampling_time`          |     `double`    |                                          Sampling time                                         |    Yes    |
     * |   `UKF`   |           `alpha`              |     `double`    |                        `alpha` parameter of the unscented kalman filter                        |    Yes    |
     * |   `UKF`   |           `beta`               |     `double`    |                        `beta` parameter of the unscented kalman filter                         |    Yes    |
     * |   `UKF`   |           `kappa`              |     `double`    |                        `kappa` parameter of the unscented kalman filter                        |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
    * @brief Finalize the estimator.
    * @param stateVariableHandler reference to a `System::VariablesHandler` object describing the ukf state.
    * @param measurementVariableHandler reference to a `System::VariablesHandler` object describing the ukf measurement.
    * @param kinDynFullModel a pointer to an iDynTree::KinDynComputations object.
    * @note You should call this method after initialized the estimator and created the UkfState model
    * which builds the handler needed in input to this method.
    * @return true in case of success, false otherwise.
    */
    bool finalize(const System::VariablesHandler& stateVariableHandler,
                  const System::VariablesHandler& measurementVariableHandler,
                  std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel);

    /**
     * @brief build a robot dynamics estimator implementing an unscented kalman filter.
     * @param handler pointer to the IParametersHandler interface.
     * @param kinDynFullModel a pointer to an iDynTree::KinDynComputations object that will be shared among
     * all the dynamics.
     * @param subModelList a list of SubModel objects
     * @param kinDynWrapperList a list of pointers to a `KinDynWrapper` objects that will be shared among
     * all the dynamics
     * @return a RobotDynamicsEstimator. In case of issues an invalid RobotDynamicsEstimator
     * will be returned.
     */
    static std::unique_ptr<RobotDynamicsEstimator>
    build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
          std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
          const std::vector<SubModel>& subModelList,
          const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList);

    /**
     * @brief set the initial state of the estimator.
     * @param initialState a reference to an `Output` object.
     * @return true in case of success, false otherwise.
     */
    bool setInitialState(const RobotDynamicsEstimatorOutput& initialState);

    /**
     * @brief Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * @brief Advance the internal state. This may change the value retrievable from getOutput().
     * @return True if the advance is successfull.
     */
    bool advance() override;

    /**
     * Set the input for the estimator.
     * @param input is a struct containing the input of the estimator.
     */
    bool setInput(const RobotDynamicsEstimatorInput& input) override;

    /**
     * Get the output of the ukf
     * @return A struct containing the ukf estimation result.
     */
    const RobotDynamicsEstimatorOutput& getOutput() const override;

}; // class RobotDynamicsEstimator

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_ROBOT_DYNAMICS_ESTIMATOR_H
