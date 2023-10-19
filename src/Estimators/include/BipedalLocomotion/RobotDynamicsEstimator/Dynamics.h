/**
 * @file Dynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_DYNAMICS_H

#include <memory>
#include <string>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/Factory.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>

/**
 * BLF_REGISTER_UKF_DYNAMICS is a macro that can be used to register a Dynamics. The key of
 * the dynamics will be the stringified version of the Dynamics C++ Type
 * @param _model the model of the dynamics
 * @param _baseModel the base model from which the _model inherits.
 */
#define BLF_REGISTER_UKF_DYNAMICS(_model, _baseModel)               \
    static std::shared_ptr<_baseModel> _model##FactoryBuilder() \
{                                                               \
    return std::make_shared<_model>();                          \
    };                                                          \
    static std::string _model##BuilderAutoRegHook               \
    = ::BipedalLocomotion::Estimators::                         \
    RobotDynamicsEstimator::DynamicsFactory::registerBuilder    \
    (#_model, _model##FactoryBuilder);

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * @brief The UKFInput struct represents the input of the ukf needed to update the dynamics.
 */
struct UKFInput
{
    Eigen::VectorXd robotJointPositions; /**< Vector of joint positions. */
    Eigen::VectorXd robotJointAccelerations; /**< Vector of joint accelerations. */
    manif::SE3d robotBasePose; /**< Robot base position and orientation. */
    manif::SE3d::Tangent robotBaseVelocity; /**< Robot base velocity. */
    manif::SE3d::Tangent robotBaseAcceleration; /**< Robot base acceleration. */
};

/**
 * UkfInputProvider describes the provider for the inputs of the ukf
 */
class UkfInputProvider : public System::Advanceable<UKFInput, UKFInput>
{
private:
    UKFInput m_ukfInput;

public:
    /**
     * Get the input
     * @return A struct containing the inputs for the ukf
     */
    const UKFInput& getOutput() const override;

    /**
     * Set the state which represents the state of the provider.
     * @param input is a struct containing the input of the ukf.
     */
    bool setInput(const UKFInput& input) override;

    /**
     * @brief Advance the internal state. This may change the value retrievable from getOutput().
     * @return True if the advance is successfull.
     */
    bool advance() override;

    /**
     * @brief Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const override;

}; // class UkfProcessInputProvider

/**
 * The class Dynamics represents a base class describing a generic model composing the ukf process
 * model or the ukf measurement model. The Dynamics object is created from a parameter handler which
 * specifies the name of the variable described by the dynamics object, the covariances associated to the variable,
 * the initial covariance, the Dynamics model which specifies which implementation of this class will be used.
 * The model of the Dynamics object depends on the current state and an input object which need to be set
 * before calling the update method.
 */
class Dynamics
{
protected:
    int m_size; /**< Size of the variable associate to the Dynamics object. */
    Eigen::VectorXd m_updatedVariable; /**< Updated variable computed using the dynamic model. */
    std::string m_description{"Generic Dynamics Element"}; /**< String describing the type of the dynamics */
    Eigen::VectorXd m_covariances; /**< Vector of covariances. */
    bool m_isInitialized{false}; /**< True if the dynamics has been initialized. */
    Eigen::VectorXd m_initialCovariances; /**< Vector of initial covariances. */

    /**
      * Controls whether the variable handler contains the variables on which the dynamics depend.
      * @return True in case all the dependencies are contained in the variable handler, false otherwise.
      */
    virtual bool checkStateVariableHandler();

public:
    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @return True in case of success, false otherwise.
     */
    virtual bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler);

    /**
     * Finalize the Dynamics.
     * @param handler object describing the variables in the vector for which all the dynamics are defined.
     * @note You should call this method after you add ALL the dynamics to the variable handler.
     * @return true in case of success, false otherwise.
     */
    virtual bool finalize(const System::VariablesHandler& handler);

    /**
     * Set the KinDynWrapper object.
     * @param subModelList list of SubModel objects
     * @param kinDynWrapperList list of pointers to KinDynWrapper objects.
     * @return True in case of success, false otherwise.
     */
    virtual bool setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList);

    /**
     * Update the dynamics of the variable.
     * @return True in case of success, false otherwise.
     */
    virtual bool update();

    /**
     * Get the next value m_updatedVariable.
     * @return a const reference to the next variable value.
     */
    Eigen::Ref<const Eigen::VectorXd> getUpdatedVariable() const;

    /**
     * Get the size of the state.
     * @return the size of the state.
     */
    int size() const;

    /**
     * Set the state of the ukf needed to update the dynamics.
     * @param ukfState reference to the ukf state.
     */
    virtual void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) = 0;

    /**
     * Set a `UKFInput` object.
     * @param ukfInput reference to the UKFInput struct.
     */
    virtual void setInput(const UKFInput & ukfInput) = 0;

    /**
     * @brief getCovariance access the covariance `Eigen::VectorXd` associated to the variables described by this dynamics.
     * @return the vector of covariances.
     */
    Eigen::Ref<const Eigen::VectorXd> getCovariance();

    /**
     * @brief getInitialStateCovariance access the covariance `Eigen::VectorXd` associated to the initial state.
     * @return the vector of covariances.
     */
    Eigen::Ref<const Eigen::VectorXd> getInitialStateCovariance();

    /**
     * Destructor.
     */
    virtual ~Dynamics() = default;
};

/**
 * DynamicsFactory implements the factory design patter for constructing a Dynamics given
 * its model.
 */
class DynamicsFactory : public System::Factory<Dynamics>
{

public:
    virtual ~DynamicsFactory() = default;
};

} // RobotDynamicsEstimator
} // Estimators
} // BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_DYNAMICS_H
