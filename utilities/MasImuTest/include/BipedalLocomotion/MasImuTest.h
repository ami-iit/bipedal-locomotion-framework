/**
 * @file MasImuTest.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
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
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>

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
        matioCpp::File outputFile;
    };

    class MasImuData
    {
        std::string m_testName;
        std::shared_ptr<CommonData> m_commonDataPtr;
        BipedalLocomotion::ParametersHandler::YarpImplementation::shared_ptr m_group;
        iDynTree::FrameIndex m_frame;
        std::string m_frameName, m_imuName, m_gyroName, m_accName;
        iDynTree::LinkIndex m_link;
        std::vector<iDynTree::LinkIndex> m_consideredJointIndexes;
        std::vector<std::string> m_consideredJointNames;
        iDynTree::KinDynComputations m_kinDyn;
        yarp::dev::PolyDriver m_orientationDriver, m_robotDriver;
        yarp::dev::IOrientationSensors* m_orientationInterface;
        yarp::dev::IThreeAxisGyroscopes* m_gyroInterface;
        yarp::dev::IThreeAxisLinearAccelerometers* m_accInterface;
        yarp::dev::IEncodersTimed* m_encodersInterface;
        size_t m_imuSensorIndex, m_gyroSensorIndex, m_accSensorIndex;
        yarp::sig::Vector m_positionFeedbackDeg; /**< Current joint position [deg]. */
        yarp::sig::Vector m_rpyInDeg, m_gyroInDeg_s, m_acc;
        iDynTree::Vector3 m_rpyInRad;
        iDynTree::VectorDynSize m_positionFeedbackInRad;
        iDynTree::VectorDynSize m_previousPositionFeedbackInRad;
        iDynTree::VectorDynSize m_dummyVelocity;
        iDynTree::Rotation m_rotationFeedback;

        iDynTree::Rotation m_rotationFromEncoders;
        iDynTree::Rotation m_imuWorld; //i_R_imuworld

        std::vector<iDynTree::Rotation> m_errorData;
        std::vector<iDynTree::VectorDynSize> m_jointsPositionData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackInInertialData;
        std::vector<iDynTree::Rotation> m_rotationFeedbackInInertialYawFilteredData;
        std::vector<yarp::sig::Vector> m_rpyImuData;
        std::vector<iDynTree::Rotation> m_rotationFromEncodersData;
        std::vector<yarp::sig::Vector> m_gyroData;
        std::vector<yarp::sig::Vector> m_accData;

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

        bool setup(const std::string& testName,
                   BipedalLocomotion::ParametersHandler::YarpImplementation::shared_ptr group,
                   std::shared_ptr<CommonData> commonDataPtr);

        bool firstRun();

        bool addSample();

        size_t addedSamples() const;

        bool isCompleted() const;

        void setCompleted();

        void printResults() const;

        bool saveResults();

        void reset();

        bool close();
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
    MasImuData m_leftIMU, m_rightIMU;
    State m_state{State::STARTED};
    std::mutex m_mutex;
    yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */

    void reset();

    void printResultsPrivate() const;

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
