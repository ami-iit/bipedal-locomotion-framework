/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */
 
#include <BipedalLocomotion/FTIMULoggerDevice.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>
#include <yarp/os/LogStream.h>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion;

FTIMULoggerDevice::FTIMULoggerDevice(double period,
                                     yarp::os::ShouldUseSystemClock useSystemClock)
: yarp::os::PeriodicThread(period, useSystemClock)
{
}


FTIMULoggerDevice::FTIMULoggerDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

FTIMULoggerDevice::~FTIMULoggerDevice()
{
}

bool FTIMULoggerDevice::open(yarp::os::Searchable& config)
{
    YarpUtilities::getElementFromSearchable(config, "robot", m_robot);
    YarpUtilities::getElementFromSearchable(config, "port_prefix", m_portPrefix);

    double devicePeriod{0.01};

    if (YarpUtilities::getElementFromSearchable(config, "sampling_period_in_s", devicePeriod))
    {
        setPeriod(devicePeriod);
    }

    if (!setupRobotSensorBridge(config))
    {
        return false;
    }

    m_ftimupair["r_leg"] = FTIMUPair();
    m_ftimupair["l_leg"] = FTIMUPair();
    m_ftimupair["l_foot"] = FTIMUPair();
    m_ftimupair["r_foot"] = FTIMUPair();

    return true;
}

bool FTIMULoggerDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        yError() << "[FTIMULoggerDevice][setupRobotSensorBridge] Missing required group \"RobotSensorBridge\"";
        return false;
    }

    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(originalHandler))
    {
        yError() << "[FTIMULoggerDevice][setupRobotSensorBridge] Could not configure RobotSensorBridge";
        return false;
    }

    return true;
}


bool FTIMULoggerDevice::attachAll(const yarp::dev::PolyDriverList & poly)
{
    if (!m_robotSensorBridge->setDriversList(poly))
    {
        yError() << "[FTIMULoggerDevice][attachAll] Could not attach drivers list to sensor bridge";
        return false;
    }

    start();
    return true;
}


void FTIMULoggerDevice::run()
{
    if (!m_robotSensorBridge->advance())
    {
        yError() << "[FTIMULoggerDevice][run] could not advance sensor bridge.";
    }

    bool ok{true};
    auto bufferSize = time.rows();

    // left leg ft
    ok = ok && m_robotSensorBridge->getSixAxisForceTorqueMeasurement("l_leg_ft_sensor", ft, timeNow);
    ok = ok && m_robotSensorBridge->getLinearAccelerometerMeasurement("l_upper_leg_ft_acc_3b12", acc, timeNow);
    ok = ok && m_robotSensorBridge->getGyroscopeMeasure("l_upper_leg_ft_gyro_3b12", gyro, timeNow);
    ok = ok && m_robotSensorBridge->getOrientationSensorMeasurement("l_upper_leg_ft_eul_3b12", orient, timeNow);
    m_ftimupair.at("l_leg").ft.conservativeResize(bufferSize+1, 6);
    m_ftimupair.at("l_leg").acc.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_leg").gyro.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_leg").orient.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_leg").ft.row(bufferSize) << ft(0), ft(1), ft(2), ft(3), ft(4), ft(5);
    m_ftimupair.at("l_leg").acc.row(bufferSize) << acc(0), acc(1), acc(2);
    m_ftimupair.at("l_leg").gyro.row(bufferSize) << gyro(0), gyro(1), gyro(2);
    m_ftimupair.at("l_leg").orient.row(bufferSize) << orient(0), orient(1), orient(2);

    // left foot ft
    ok = ok && m_robotSensorBridge->getSixAxisForceTorqueMeasurement("l_foot_ft_sensor", ft, timeNow);
    ok = ok && m_robotSensorBridge->getLinearAccelerometerMeasurement("l_foot_ft_acc_3b13", acc, timeNow);
    ok = ok && m_robotSensorBridge->getGyroscopeMeasure("l_foot_ft_gyro_3b13", gyro, timeNow);
    ok = ok && m_robotSensorBridge->getOrientationSensorMeasurement("l_foot_ft_eul_3b13", orient, timeNow);
    m_ftimupair.at("l_foot").ft.conservativeResize(bufferSize+1, 6);
    m_ftimupair.at("l_foot").acc.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_foot").gyro.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_foot").orient.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("l_foot").ft.row(bufferSize) << ft(0), ft(1), ft(2), ft(3), ft(4), ft(5);
    m_ftimupair.at("l_foot").acc.row(bufferSize) << acc(0), acc(1), acc(2);
    m_ftimupair.at("l_foot").gyro.row(bufferSize) << gyro(0), gyro(1), gyro(2);
    m_ftimupair.at("l_foot").orient.row(bufferSize) << orient(0), orient(1), orient(2);

    // right leg ft
    ok = ok && m_robotSensorBridge->getSixAxisForceTorqueMeasurement("r_leg_ft_sensor", ft, timeNow);
    ok = ok && m_robotSensorBridge->getLinearAccelerometerMeasurement("r_upper_leg_ft_acc_3b11", acc, timeNow);
    ok = ok && m_robotSensorBridge->getGyroscopeMeasure("r_upper_leg_ft_gyro_3b11", gyro, timeNow);
    ok = ok && m_robotSensorBridge->getOrientationSensorMeasurement("r_upper_leg_ft_eul_3b11", orient, timeNow);
    m_ftimupair.at("r_leg").ft.conservativeResize(bufferSize+1, 6);
    m_ftimupair.at("r_leg").acc.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_leg").gyro.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_leg").orient.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_leg").ft.row(bufferSize) << ft(0), ft(1), ft(2), ft(3), ft(4), ft(5);
    m_ftimupair.at("r_leg").acc.row(bufferSize) << acc(0), acc(1), acc(2);
    m_ftimupair.at("r_leg").gyro.row(bufferSize) << gyro(0), gyro(1), gyro(2);
    m_ftimupair.at("r_leg").orient.row(bufferSize) << orient(0), orient(1), orient(2);

    // right foot ft
    ok = ok && m_robotSensorBridge->getSixAxisForceTorqueMeasurement("r_foot_ft_sensor", ft, timeNow);
    ok = ok && m_robotSensorBridge->getLinearAccelerometerMeasurement("r_foot_ft_acc_3b14", acc, timeNow);
    ok = ok && m_robotSensorBridge->getGyroscopeMeasure("r_foot_ft_gyro_3b14", gyro, timeNow);
    ok = ok && m_robotSensorBridge->getOrientationSensorMeasurement("r_foot_ft_eul_3b14", orient, timeNow);
    m_ftimupair.at("r_foot").ft.conservativeResize(bufferSize+1, 6);
    m_ftimupair.at("r_foot").acc.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_foot").gyro.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_foot").orient.conservativeResize(bufferSize+1, 3);
    m_ftimupair.at("r_foot").ft.row(bufferSize) << ft(0), ft(1), ft(2), ft(3), ft(4), ft(5);
    m_ftimupair.at("r_foot").acc.row(bufferSize) << acc(0), acc(1), acc(2);
    m_ftimupair.at("r_foot").gyro.row(bufferSize) << gyro(0), gyro(1), gyro(2);
    m_ftimupair.at("r_foot").orient.row(bufferSize) << orient(0), orient(1), orient(2);

    time.conservativeResize(bufferSize+1);
    time.row(bufferSize) << timeNow;

    if (!ok)
    {
        yError() << "[FTIMULoggerDevice][run] error reading one fo the sensors.";
    }

}

bool FTIMULoggerDevice::logData()
{
    matioCpp::File file = matioCpp::File::Create("ftimu-out.mat");
    auto outTime = Conversions::tomatioCpp(time, "time");

    // left leg ft
    matioCpp::MultiDimensionalArray<double> outLFTLeg{"l_leg_ft_sensor",
                                                      {static_cast<std::size_t>(m_ftimupair.at("l_leg").ft.rows()),
                                                      static_cast<std::size_t>(m_ftimupair.at("l_leg").ft.cols())},
                                                      m_ftimupair.at("l_leg").ft.data()};
    matioCpp::MultiDimensionalArray<double> outLFTAccLeg{"l_leg_ft_acc",
                                                         {static_cast<std::size_t>(m_ftimupair.at("l_leg").acc.rows()),
                                                         static_cast<std::size_t>(m_ftimupair.at("l_leg").acc.cols())},
                                                         m_ftimupair.at("l_leg").acc.data()};
    matioCpp::MultiDimensionalArray<double> outLFTGyroLeg{"l_leg_ft_gyro",
                                                          {static_cast<std::size_t>(m_ftimupair.at("l_leg").gyro.rows()),
                                                          static_cast<std::size_t>(m_ftimupair.at("l_leg").gyro.cols())},
                                                          m_ftimupair.at("l_leg").gyro.data()};
    matioCpp::MultiDimensionalArray<double> outLFTOrientLeg{"l_leg_ft_orient",
                                                            {static_cast<std::size_t>(m_ftimupair.at("l_leg").orient.rows()),
                                                            static_cast<std::size_t>(m_ftimupair.at("l_leg").orient.cols())},
                                                            m_ftimupair.at("l_leg").orient.data()};

    std::vector<matioCpp::Variable> l_leg_ftimu;
    l_leg_ftimu.emplace_back(outLFTLeg);
    l_leg_ftimu.emplace_back(outLFTAccLeg);
    l_leg_ftimu.emplace_back(outLFTGyroLeg);
    l_leg_ftimu.emplace_back(outLFTOrientLeg);
    matioCpp::Struct outLL("l_leg_ft_imu", l_leg_ftimu);

    // right leg ft
    matioCpp::MultiDimensionalArray<double> outRFTLeg{"r_leg_ft_sensor",
                                                      {static_cast<std::size_t>(m_ftimupair.at("r_leg").ft.rows()),
                                                      static_cast<std::size_t>(m_ftimupair.at("r_leg").ft.cols())},
                                                      m_ftimupair.at("r_leg").ft.data()};
    matioCpp::MultiDimensionalArray<double> outRFTAccLeg{"r_leg_ft_acc",
                                                         {static_cast<std::size_t>(m_ftimupair.at("r_leg").acc.rows()),
                                                         static_cast<std::size_t>(m_ftimupair.at("r_leg").acc.cols())},
                                                         m_ftimupair.at("r_leg").acc.data()};
    matioCpp::MultiDimensionalArray<double> outRFTGyroLeg{"r_leg_ft_gyro",
                                                          {static_cast<std::size_t>(m_ftimupair.at("r_leg").gyro.rows()),
                                                          static_cast<std::size_t>(m_ftimupair.at("r_leg").gyro.cols())},
                                                          m_ftimupair.at("r_leg").gyro.data()};
    matioCpp::MultiDimensionalArray<double> outRFTOrientLeg{"r_leg_ft_orient",
                                                            {static_cast<std::size_t>(m_ftimupair.at("r_leg").orient.rows()),
                                                            static_cast<std::size_t>(m_ftimupair.at("r_leg").orient.cols())},
                                                            m_ftimupair.at("r_leg").orient.data()};

    std::vector<matioCpp::Variable> r_leg_ftimu;
    r_leg_ftimu.emplace_back(outRFTLeg);
    r_leg_ftimu.emplace_back(outRFTAccLeg);
    r_leg_ftimu.emplace_back(outRFTGyroLeg);
    r_leg_ftimu.emplace_back(outRFTOrientLeg);
    matioCpp::Struct outRL("r_leg_ft_imu", r_leg_ftimu);

    // left foot ft
    matioCpp::MultiDimensionalArray<double> outLFTFoot{"l_foot_ft_sensor",
                                                      {static_cast<std::size_t>(m_ftimupair.at("l_foot").ft.rows()),
                                                       static_cast<std::size_t>(m_ftimupair.at("l_foot").ft.cols())},
                                                       m_ftimupair.at("l_foot").ft.data()};
    matioCpp::MultiDimensionalArray<double> outLFTAccFoot{"l_leg_ft_acc",
                                                         {static_cast<std::size_t>(m_ftimupair.at("l_foot").acc.rows()),
                                                          static_cast<std::size_t>(m_ftimupair.at("l_foot").acc.cols())},
                                                          m_ftimupair.at("l_foot").acc.data()};
    matioCpp::MultiDimensionalArray<double> outLFTGyroFoot{"l_foot_ft_gyro",
                                                          {static_cast<std::size_t>(m_ftimupair.at("l_foot").gyro.rows()),
                                                           static_cast<std::size_t>(m_ftimupair.at("l_foot").gyro.cols())},
                                                           m_ftimupair.at("l_foot").gyro.data()};
    matioCpp::MultiDimensionalArray<double> outLFTOrientFoot{"l_foot_ft_orient",
                                                            {static_cast<std::size_t>(m_ftimupair.at("l_foot").orient.rows()),
                                                             static_cast<std::size_t>(m_ftimupair.at("l_foot").orient.cols())},
                                                             m_ftimupair.at("l_foot").orient.data()};

    std::vector<matioCpp::Variable> l_foot_ftimu;
    l_foot_ftimu.emplace_back(outLFTFoot);
    l_foot_ftimu.emplace_back(outLFTAccFoot);
    l_foot_ftimu.emplace_back(outLFTGyroFoot);
    l_foot_ftimu.emplace_back(outLFTOrientFoot);
    matioCpp::Struct outLF("l_foot_ft_imu", l_foot_ftimu);


    // right foot ft
    matioCpp::MultiDimensionalArray<double> outRFTFoot{"r_foot_ft_sensor",
                                                      {static_cast<std::size_t>(m_ftimupair.at("r_foot").ft.rows()),
                                                       static_cast<std::size_t>(m_ftimupair.at("r_foot").ft.cols())},
                                                       m_ftimupair.at("r_foot").ft.data()};
    matioCpp::MultiDimensionalArray<double> outRFTAccFoot{"r_leg_ft_acc",
                                                         {static_cast<std::size_t>(m_ftimupair.at("r_foot").acc.rows()),
                                                          static_cast<std::size_t>(m_ftimupair.at("r_foot").acc.cols())},
                                                          m_ftimupair.at("r_foot").acc.data()};
    matioCpp::MultiDimensionalArray<double> outRFTGyroFoot{"r_foot_ft_gyro",
                                                          {static_cast<std::size_t>(m_ftimupair.at("r_foot").gyro.rows()),
                                                           static_cast<std::size_t>(m_ftimupair.at("r_foot").gyro.cols())},
                                                           m_ftimupair.at("r_foot").gyro.data()};
    matioCpp::MultiDimensionalArray<double> outRFTOrientFoot{"r_foot_ft_orient",
                                                            {static_cast<std::size_t>(m_ftimupair.at("r_foot").orient.rows()),
                                                             static_cast<std::size_t>(m_ftimupair.at("r_foot").orient.cols())},
                                                             m_ftimupair.at("r_foot").orient.data()};

    std::vector<matioCpp::Variable> r_foot_ftimu;
    r_foot_ftimu.emplace_back(outRFTFoot);
    r_foot_ftimu.emplace_back(outRFTAccFoot);
    r_foot_ftimu.emplace_back(outRFTGyroFoot);
    r_foot_ftimu.emplace_back(outRFTOrientFoot);
    matioCpp::Struct outRF("r_foot_ft_imu", r_foot_ftimu);

    bool write_ok{true};
    write_ok = write_ok && file.write(outLL);
    write_ok = write_ok && file.write(outRL);
    write_ok = write_ok && file.write(outLF);
    write_ok = write_ok && file.write(outRF);
    write_ok = write_ok && file.write(outTime);

    if (!write_ok)
    {
        yError() <<  "[FTIMULoggerDevice][logData] Could not write to file." ;
        return false;
    }

    return true;
}

bool FTIMULoggerDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}


bool FTIMULoggerDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (!logData())
    {
        yError() << "[FTIMULoggerDevice][close] Failed to log data.";
    }

    return true;
}
