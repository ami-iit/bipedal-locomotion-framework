/**
 * @file MasImuTest.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H

// STD
#include <string>
#include <vector>
#include <mutex>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IEncodersTimed.h>

// iDynTree
#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/Transform.h>

// Eigen
#include <Eigen/Core>

// matioCpp
#include <matioCpp/matioCpp.h>

//Thrifts
#include <thrifts/MasImuTestCommands.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

namespace BipedalLocomotion
{
    class MasImuTest;
}

class BipedalLocomotion::MasImuTest : public yarp::os::RFModule, public MasImuTestCommands {

    struct CommonData
    {
        std::string robotName;
        std::string prefix;
        iDynTree::Model fullModel;
        iDynTree::Traversal traversal;
        iDynTree::Transform baseTransform;
        bool filterYaw;
        int maxSamples;
        double minJointVariationRad;
        double masTimeout;
    };

    class MasImuData
    {
        std::string m_testName, m_logPrefix;
        std::shared_ptr<CommonData> m_commonDataPtr;
        BipedalLocomotion::ParametersHandler::YarpImplementation::shared_ptr m_group;
        iDynTree::FrameIndex m_frame;
        std::string m_frameName, m_imuName, m_gyroName, m_accName;
        std::vector<iDynTree::LinkIndex> m_consideredJointIndexes;
        std::vector<std::string> m_consideredJointNames;
        iDynTree::KinDynComputations m_kinDyn;
        yarp::dev::PolyDriver m_orientationDriver, m_robotDriver;
        yarp::dev::IOrientationSensors* m_orientationInterface;
        yarp::dev::IThreeAxisGyroscopes* m_gyroInterface;
        yarp::dev::IThreeAxisLinearAccelerometers* m_accInterface;
        yarp::dev::IEncodersTimed* m_encodersInterface;
        size_t m_imuSensorIndex, m_gyroSensorIndex, m_accSensorIndex;
        yarp::sig::Vector m_positionFeedbackDeg;
        yarp::sig::Vector m_rpyInDeg, m_gyroInDeg_s, m_acc;
        Eigen::Vector3d m_rpyInDegRemapped;
        iDynTree::VectorDynSize m_positionFeedbackInRad;
        iDynTree::VectorDynSize m_previousPositionFeedbackInRad;
        iDynTree::VectorDynSize m_dummyVelocity;
        iDynTree::Rotation m_rotationFeedback;
        Eigen::Matrix3d m_rpyMapping;

        iDynTree::Rotation m_rotationFromEncoders;
        iDynTree::Rotation m_imuWorld; //i_R_imuworld

        std::vector<iDynTree::Rotation> m_errorData;
        std::vector<iDynTree::VectorDynSize> m_jointsPositionData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackInInertialData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackInInertialYawFilteredData;
        std::vector<yarp::sig::Vector> m_rpyImuData;
        std::vector<Eigen::Vector3d> m_rpyRemappedData;
        std::vector<iDynTree::Rotation> m_rotationFromEncodersData;
        std::vector<yarp::sig::Vector> m_gyroData;
        std::vector<yarp::sig::Vector> m_accData;

        std::string m_output;

        bool m_completed{false};

        void reserveData();

        void clearData();

        bool setupModel();

        bool setupIMUDriver();

        bool setupOrientationSensor();

        bool setupGyro();

        bool setupAccelerometer();

        bool setupEncoders();

        bool getFeedback();

        bool updateRotationFromEncoders();

        double maxVariation();

    public:

        bool setup(BipedalLocomotion::ParametersHandler::YarpImplementation::shared_ptr group,
                   std::shared_ptr<CommonData> commonDataPtr);

        bool firstRun();

        bool addSample();

        size_t addedSamples() const;

        bool isCompleted() const;

        void setCompleted();

        std::string printResults();

        bool saveResults(matioCpp::Struct &logStruct);

        void reset();

        bool close();

        const std::string& name() const;
    };

    enum class State
    {
        STARTED,
        PREPARED,
        FIRST_RUN,
        RUNNING
    };

    BipedalLocomotion::ParametersHandler::YarpImplementation::shared_ptr m_parametersPtr;
    std::shared_ptr<CommonData> m_commonDataPtr;
    double m_period;
    std::vector<std::unique_ptr<MasImuData>> m_tests;
    State m_state{State::STARTED};
    std::mutex m_mutex;
    yarp::os::Port m_rpcPort;
    matioCpp::File m_outputFile;

    void reset();

    void printResultsPrivate();

    void saveResultsPrivate();

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module in seconds.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * Call this method to start the test.
     * @return true/false in case of success/failure (for example if the preparation phase was not successfull);
     */
    bool startTest() override;

    /**
     * Stop the test
     */
    bool stopTest() override;

    /**
     * Print the results. This works only if the test has already been stopped.
     */
    bool printResults() override;

    /**
     * Quits the module
     */
    void quit() override;
};

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H
