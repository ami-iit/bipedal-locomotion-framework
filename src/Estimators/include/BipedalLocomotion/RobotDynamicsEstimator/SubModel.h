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
    std::string associatedJoint; /**< Name of the fixd joint used to represent the ft sensor in the model. */
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
     * Determines the validity of the object.
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const;

    /**
     * Getters
     */

    /**
     * @brief Get the `iDynTree::Model` instance.
     * @note The actual implementation of the Model is currently stored in an `iDynTree::Model`
     * @return The model of the SubModel.
     */
    const iDynTree::Model& getModel() const;

    /**
     * @brief Access the `std::vectot<int>` list.
     * @return the mapping between the joint indeces in the sub-model and the joint indeces in the
     * full-model.
     */
    const std::vector<int>& getJointMapping() const;

    /**
     * @brief Access the `std::vector<FT>` list.
     * @return the list of FT objects which is the list of force/torque sensors.
     */
    const std::vector<FT>& getFTList() const;

    /**
     * @brief Access the `std::vector<Sensor>` list of acceletometer sensors.
     * @return a list of Sensor objects describing the accelerometers contained in the sub-model.
     */
    const std::vector<Sensor>& getAccelerometerList() const;

    /**
     * @brief Access the `std::vector<Sensor>` list of gyroscope sensors.
     * @return a list of Sensor objects describing the gyroscope contained in the sub-model.
     */
    const std::vector<Sensor>& getGyroscopeList() const;

    /**
     * @brief Access the `std::vector<std::string>` list of frame names.
     * @return a list of strings describing frame names of the external contacts for the sub-model.
     */
    const std::vector<std::string>& getExternalContactList() const;

    /**
     * @brief access the length of force/torque sensor list.
     * @return the number of force/torque sensors in the sub-model.
     */
    std::size_t getNrOfFTSensor() const;

    /**
     * @brief access the length of accelerometer list.
     * @return the number of accelerometer sensors in the sub-model.
     */
    std::size_t getNrOfAccelerometer() const;

    /**
     * @brief access the length of gyroscope list.
     * @return the number of gyroscope sensors in the sub-model.
     */
    std::size_t getNrOfGyroscope() const;

    /**
     * @brief access the length of the contact frame list.
     * @return the number of gyroscope sensors in the sub-model.
     */
    std::size_t getNrOfExternalContact() const;

    /**
     * @brief Access an element of the force/torque sensor list.
     * @param index is the index of the force/torque sensor in the submodel
     * @return FT object associated with the specified index.
     */
    const FT& getFTSensor(const int index) const;

    /**
     * @brief Access an element of the force/torque sensor list.
     * @param is the name of the force/torque sensor.
     * @return FT object associated with the specified name.
     */
    const FT& getFTSensor(const std::string name) const;

    /**
     * @brief hasFTSensor check if the force/torque sensor is part of the sub-model
     * @param name is the name of the ft sensor
     * @return true if the sensor is found, false otherwise
     */
    bool hasFTSensor(const std::string name) const;

    /**
     * @brief Access an element of the accelerometer list.
     * @param index is the index of the accelerometer in the submodel
     * @return a Sensor object corresponding to the accelerometer associated with the specified index.
     */
    const Sensor& getAccelerometer(const int index) const;

    /**
     * @brief Access an element of the accelerometer list.
     * @param is the name of the accelerometer.
     * @return a Sensor object corresponding to the accelerometer associated with the specified name.
     */
    const Sensor& getAccelerometer(const std::string name) const;

    /**
     * @brief hasAccelerometer check if the accelerometer is part of the sub-model
     * @param name is the name of the accelerometer
     * @return true if the sensor is found, false otherwise
     */
    bool hasAccelerometer(const std::string name) const;

    /**
     * @brief Access an element of the gyroscope list.
     * @param index is the index of the gyroscope in the submodel
     * @return a Sensor object corresponding to the gyroscope associated with the specified index.
     */
    const Sensor& getGyroscope(const int index) const;

    /**
     * @brief Access an element of the gyroscope list.
     * @param is the name of the gyroscoper.
     * @return a Sensor object corresponding to the gyroscope associated with the specified name.
     */
    const Sensor& getGyroscope(const std::string name) const;

    /**
     * @brief hasAccelerometer check if the gyroscope is part of the sub-model
     * @param name is the name of the gyroscope
     * @return true if the sensor is found, false otherwise
     */
    bool hasGyroscope(const std::string name) const;

    /**
     * @brief access an element of the contact frame list.
     * @param index is the index of the external contact in the submodel.
     * @return a string corresponding to the external contact frame associated with the specified index.
     */
    const std::string& getExternalContact(const int index) const;

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
     * @param ftSensorList list of FT structs.
     * @param accList list of Sensor structs.
     * @param gyroList list of Sensor structs.
     * @param externalContacts list of strings.
     * @return a boolean value saying if the subModelList has been created correctly.
     */
    bool createSubModels(const std::vector<FT>& ftSensorList,
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
     * @brief Set model which is an instance of `iDynTree::ModelLoader` and the sensor list which is an instance of `iDynTree::SensorList`
     * @param modelLoader is an iDynTree ModelLoader object
     * @param sensors is an iDynTree SensorList object
     */
    void setModelAndSensors(const iDynTree::Model& model, const iDynTree::SensorsList& sensors);

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
     * @brief access the length of the list std::vector<SubModel>.
     * @return the number of sub-models composing the model
     */
    std::size_t getNrOfSubModels() const;

    /**
     * @brief get the `std::vector<SubModel>` list.
     * @return the list of SubModel objects.
     */
    const std::vector<SubModel>& getSubModelList() const;

    /**
     * @brief get a `SubModel` instance of the list of `SubModel`.
     * @return the SubModel at the position index.
     */
    const SubModel& getSubModel(int index) const;
};

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_SUBMODEL_H
