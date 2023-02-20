/**
 * @file SubModel.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_SUBMODEL_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_SUBMODEL_H

#include <memory>
#include <string>
#include <vector>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// BLF
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * Sensor describes the generic sensors of the submodels.
 */
struct Sensor
{
    std::string name; /**< Name of the sensor in the URDF */
    std::string frame; /**< Frame of the sensor in the URDF */
};

/**
 * FT describes the force/torque sensors of the submodels.
 */
struct FT : Sensor
{
    /**
     * Direction can assume values:
     * 1 if the system is applying the force on the sub-model.
     * -1 if the sub-model is applying the force on the system.
     * 0 if the direction is not specified.
     */
    enum class Direction : int
    {
        Positive = 1,
        Negative = -1,
        NotSpecified = 0
    };

    Direction forceDirection = Direction::NotSpecified; /**< Force direction depending on which side
                                                           of the sensor is considered (+1 or -1)*/
};

/**
 * SubModel is a concrete class describing the sub-model object, its model, the list of sensors and
 * the mapping between its joints and the same joints in the full model.
 */
class SubModel
{
    iDynTree::Model m_model; /**< iDynTree Model object describing the submodel */
    std::vector<int> m_jointListMapping; /**< Each element contains an index describing the joint
                                            position in the full model*/
    std::vector<FT> m_ftList; /**< List of force/torque sensors in the submodel */
    std::vector<Sensor> m_accelerometerList; /**< List of accelerometers in the submodel */
    std::vector<Sensor> m_gyroscopeList; /**< List of gyroscopes in the submodel */
    std::vector<std::string> m_externalContactList; /**< List of the additional external contacts */

public:
    /**
     * Getters
     */

    /**
     * @brief Access model.
     * @return The model of the SubModel.
     */
    const iDynTree::Model& getModel() const;

    /**
     * @brief Access jointListMapping.
     * @return the mapping between the joint indeces in the sub-model and the joint indeces in the
     * full-model.
     */
    const std::vector<int>& getJointMapping() const;

    /**
     * @brief Access ftList.
     * @return the list of FT objects which is the list of force/torque sensors.
     */
    const std::vector<FT>& getFTList() const;

    /**
     * @brief Access accelerometerList.
     * @return a list of Sensor objects describing the accelerometers contained in the sub-model.
     */
    const std::vector<Sensor>& getAccelerometerList() const;

    /**
     * @brief Access gyroscopeList.
     * @return a list of Sensor objects describing the gyroscope contained in the sub-model.
     */
    const std::vector<Sensor>& getGyroscopeList() const;

    /**
     * @brief Access externalContactList.
     * @return a list of strings describing frame names of the external contacts for the sub-model.
     */
    const std::vector<std::string>& getExternalContactList() const;

    friend class SubModelCreator;
};

/**
 * SubModelCreator is a concrete class and splits a model into sub-models along the force/torque
 * sensors specified as configuration parameters.
 */

class SubModelCreator
{
    iDynTree::Model m_model; /**< Model to split. */
    iDynTree::SensorsList m_sensorList; /**< List of sensors in the model. */
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */
    std::vector<SubModel> m_subModelList; /**< List of SubModel objects */

    /**
     * @brief splitModel splits the model objects in many submodel objects cutting the model at the
     * ft sensors.
     * @param ftFrameList list of strings containing the force/torque sensor names used to split the
     * model into sub-models
     * @param idynSubModels list of iDynTree Model objects..
     * @return a boolean value saying if the list of submodels is created correctly.
     */
    bool splitModel(const std::vector<std::string>& ftFrameList,
                    std::vector<iDynTree::Model>& idynSubModels);

    /**
     * @brief populateSubModel populate a submodel struct.
     * @param idynSubModel iDynTree Model describing one of the sub-models.
     * @param accList list of Sensor structs.
     * @param gyroList list of Sensor structs.
     * @param externalContacts list of strings describing the external contact frames.
     * @return a subModel structure containing all the information about joint mapping, ft sensors,
     * accelerometers, gyroscopes, external contacts.
     */
    SubModel populateSubModel(iDynTree::Model& idynSubModel,
                              const std::vector<Sensor>& accList,
                              const std::vector<Sensor>& gyroList,
                              const std::vector<std::string>& externalContacts);

    /**
     * @brief attachFTsToSubModel finds all the ft sensors connected to the specified model
     * analyzing the sensorList from the full model. Per each FT sensor creates a FT struct.
     * @param idynSubModel iDynTree Model describing one of the sub-models.
     * @return a vector of FT structs where each FT is connected to the model (idynSubModel input
     * param)
     */
    std::vector<FT> attachFTsToSubModel(iDynTree::Model& idynSubModel);

    /**
     * @brief attachAccelerometersToSubModel finds all the accelerometer sensors connected to the
     * specified model analyzing the accListFromConfig list. Per each accelerometer sensor creates a
     * Sensor struct.
     * @param accListFromConfig list of Sensor structs.
     * @param subModel iDynTree Model object describing one of the sub-models.
     * @return a vector of Sensor structs.
     */
    std::vector<Sensor> attachAccelerometersToSubModel(const std::vector<Sensor>& accListFromConfig,
                                                       const iDynTree::Model& subModel);

    /**
     * @brief attachGyroscopesToSubModel finds all the gyroscope sensors connected to the specified
     * model analyzing the gyroListFromConfig list. Per each gyroscope sensor creates a Sensor
     * struct.
     * @param gyroListFromConfig list of Sensor structs.
     * @param subModel iDynTree Model object describing one of the sub-models.
     * @return a vector of Sensor structs.
     */
    std::vector<Sensor> attachGyroscopesToSubModel(const std::vector<Sensor>& gyroListFromConfig,
                                                   const iDynTree::Model& subModel);

    /**
     * @brief attachExternalContactsToSubModel finds all the contact frames on the specified model
     * analyzing. the contactsFromConfig list
     * @param contactsFromConfig list of strings describing the external contact frames.
     * @param subModel iDynTree Model object.
     * @return a vector of strings describing the contact frame names.
     */
    std::vector<std::string>
    attachExternalContactsToSubModel(const std::vector<std::string>& contactsFromConfig,
                                     const iDynTree::Model& subModel);

    /**
     * @brief createJointMapping creates a map between the joint indeces in the submodel and the
     * same joints in the full model.
     * @param subModel iDynTree Model describing one of the sub-models.
     * @return a vector of integers where the positions of an element represents the joint index in
     * the sub-model and the value of the element represents the joint index in the model.
     */
    std::vector<int> createJointMapping(const iDynTree::Model& subModel);

public:
    /**
     * @brief createSubModels splits the model in SubModel objects cutting the model at the
     * force/torque sensors specified by the parameterHandler.
     * @param ftSensorList list of Sensor structs.
     * @param accList list of Sensor structs.
     * @param gyroList list of Sensor structs.
     * @param externalContacts list of strings.
     * @return a boolean value saying if the subModelList has been created correctly.
     */
    bool createSubModels(const std::vector<Sensor>& ftSensorList,
                         const std::vector<Sensor>& accList,
                         const std::vector<Sensor>& gyroList,
                         const std::vector<std::string>& externalContacts);

    /**
     * @brief createSubModels splits the model in SubModel objects cutting the model at the
     * force/torque sensors specified by the parameterHandler.
     * @param parameterHandler IParametersHandler object
     * @return a boolean value saying if the subModelList has been created correctly.
     */
    bool
    createSubModels(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        parameterHandler);

    /**
     * Setter
     */

    /**
     * @brief Set model
     * @param model is an iDynTree Model object
     */
    void setModel(const iDynTree::Model& model);

    /**
     * @brief Set sensorList
     * @param sensorList is an iDynTree SensorsList object containing the list of Sensors
     * (gyroscopes, accelerometers, force/torque sensors) in the model
     */
    void setSensorList(const iDynTree::SensorsList& sensorList);

    /**
     * @brief set kinDyn
     * @param kinDyn is an iDynTree KinDynComputation object handling the kinematics and the
     * dynamics of the model.
     * @return a boolean value saying if the input pointer is valid
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Getter
     */

    /**
     * @brief get subModelList
     * @return the list of SubModel objects.
     */
    const std::vector<SubModel>& getSubModelList() const;

    /**
     * @brief get subModel
     * @return the SubModel at the position index.
     */
    const SubModel& getSubModel(int index) const;
};

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_SUBMODEL_H
