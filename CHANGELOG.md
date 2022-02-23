# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
### Added
- Implement the python bindings for the clock machinery and for the yarp clock (https://github.com/ami-iit/bipedal-locomotion-framework/pull/500)
- Implement the `IWeightProvider` interface and the `ConstantWeightProvider` class in the System component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/506)
- Add installation of pip metadata files for when blf python bindings are installed only via CMake (https://github.com/ami-iit/bipedal-locomotion-framework/pull/508)
- Implement the python bindings for the VectorsCollection message and the associated buffered port (https://github.com/ami-iit/bipedal-locomotion-framework/pull/511)
- Implement the `VectorsCollectionWrapper` device for collection of arbitrary vector ports (https://github.com/ami-iit/bipedal-locomotion-framework/pull/512)

### Changed
- An error it will be returned if the user tries to change the clock type once the `clock()` has been already called once (https://github.com/ami-iit/bipedal-locomotion-framework/pull/500)
- Log the arms external wrenches on the YarpRobotLogger for iCubGenova09 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/502)
- IK and TSID now uses the weight provider to specify the weight associated to a task (https://github.com/ami-iit/bipedal-locomotion-framework/pull/506)
- The `Planners`, `System`, `RobotInterface` and `YarpImplementation` components are no more mandatory to compile the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/511)

### Fix
- Remove outdated includes in YarpRobotLoggerDevice.cpp (https://github.com/ami-iit/bipedal-locomotion-framework/pull/502)

## [0.6.0] - 2022-12-10
### Added
- Add the reading of the orientation of the head IMU in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/471)
- Add the possibility to change the weight in TSID/IK (https://github.com/ami-iit/bipedal-locomotion-framework/pull/475)
- Implement a `FirstOrderSmoother` class in `ContinuousDynamicalSystem` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/476)
- Implement `getIntegrationStep` in `FixedIntegration` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/476)
- Add the possibility to create custom `LinearTasks` in python (https://github.com/ami-iit/bipedal-locomotion-framework/pull/480)
- Implement the possibility to compute the residual terms in the `LinearTask` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/483)
- Define `VectorsCollection` message in `YarpUtilities` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/483)
- Add the reading of the joint and motor acceleration in `YarpSensorBridge` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/492)

### Changed
- Use yarp clock instead of system clock in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/473)
- Reduce code duplication in python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/484)
- Use `TextLogger` in `YarpRobotLoggerDevice` instead of `yarp` commands (https://github.com/ami-iit/bipedal-locomotion-framework/pull/486)
- Ask for `osqp-eigen 0.6.4.100`(https://github.com/ami-iit/bipedal-locomotion-framework/pull/490)
- Use enum underlying type to convert `TextLogging` verbosity level to `spdlog` verbosity level (https://github.com/ami-iit/bipedal-locomotion-framework/pull/495)
- `yarp-telemetry` is now a dependency of the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/487)
- Fix deprecated `YARP` functions in `YarpUtilities` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/491)

### Fix
- Fix the population of the jointAccelerations and baseAcceleration variables in QPTSID (https://github.com/ami-iit/bipedal-locomotion-framework/pull/478)
- Fix the documentation in the `Advanceable` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/476)
- Add virtual destrutors in `System::Sink`, `System::Source`, `System::LinearTask`,
`System::ITaskControlMode`, `TSID::TSIDLinearTask` and `IK::IKLinearTask` classes (https://github.com/ami-iit/bipedal-locomotion-framework/pull/480)
- The joint torques is now correctly retrieved in QPTSID class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/482)
- The motor velocity and positions are now returned in rad/s and rad (https://github.com/ami-iit/bipedal-locomotion-framework/pull/489)
- Fix `YarpRobotLoggerDevice` documentation (https://github.com/ami-iit/bipedal-locomotion-framework/pull/472)

## [0.5.0] - 2021-11-30
### Added
- Implement Python bindings for the TSID component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/428)
- Add the possibility to set the name of each element of a variable stored in the variables handler (https://github.com/ami-iit/bipedal-locomotion-framework/pull/429)
- Develop the python bindings for toml implementation of the parameters handler (https://github.com/ami-iit/bipedal-locomotion-framework/pull/432)
- Implement the VariableRegularizationTask in TSID (https://github.com/ami-iit/bipedal-locomotion-framework/pull/431)
- Implement `create_tsid` utility function for the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/433)
- Implement the `AngularMomentumTask` in the `TSID` component and the associated python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/436)
- Implement `QPTSID::toString` method and the associated python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/440)
- Implement `ContactWrench` python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/441)
- Implement AngularMomentum task in the IK component and the associated bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/443)
- Implement `create_ik` utility function for the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/444)
- Add the possibility to set the task controller mode for the SE3Task in the TSID component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/445)
- Expose the `ITaskControlMode` class in the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/445)
- Add the possibility to enable/disable the joints and motors state logging in the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/453)
- Implement `QPInverseKinematics::toString` method and the associated python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/461)
- Add the cartesian wrenches logging in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/447)
- Implement the python bindings for the manif conversions methods (https://github.com/ami-iit/bipedal-locomotion-framework/pull/465)

### Changed
- Inherits all the `Eigen::Matrix` constructors in the `Wrenchd` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/441)
- Bump the minimum `cmake` version to `3.16.0` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/468)

### Fix
- Fix Analog FT Sensor reading in `YarpSensorBridgeImpl` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/459)
- Fix config files in `YarpRobotLoggerDevice` for iCub3 head IMU reading (https://github.com/ami-iit/bipedal-locomotion-framework/pull/467)

## [0.4.0] - 2021-10-15
### Added
- Implement `AdvanceableRunner::isRunning()` method (https://github.com/dic-iit/bipedal-locomotion-framework/pull/395)
- Implement `ContactPhaseList::getPresentPhase()` method (https://github.com/dic-iit/bipedal-locomotion-framework/pull/396)
- Add a synchronization mechanism for the `AdvanceableRunner` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/403)
- Add the possibility to use spdlog with YARP (https://github.com/ami-iit/bipedal-locomotion-framework/pull/408)
- Add new Advanceable exposing `UnicyclePlanner` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/320)

### Changed
- Add `name` parameter to the `AdvanceableRunner` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/406)
- Set the required `spdlog` version in the cmake file (https://github.com/ami-iit/bipedal-locomotion-framework/pull/415)
- Add features to FTIMULoggerDevice and rename it in YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/405)

### Fix
- Fix missing components dependencies in the `CMake` machinery (https://github.com/ami-iit/bipedal-locomotion-framework/pull/414)
- Fixed missing include in `FloatingBaseEstimatorIO.h` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/417)

## [0.3.0] - 2021-08-12
### Added
- Implement `CubicSpline` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/344)
- Implement `PWM` control in RobotControl class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/346)
- Implement `ContactWrenchCone` class in Math component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/352)
- Implement `skew` function in Math component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/352)
- Implement `QPTSID` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/366)
- Implement motor pwm, motor encoders, wbd joint torque estimates, pid reading in `YarpSensorBridge`(https://github.com/dic-iit/bipedal-locomotion-framework/pull/359).
- Implement FeasibleContactWrenchTask for TSID component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/369).
- Implement python bindings for QPInverseKinematics class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/303)
- Implement `ControlTask` in for System component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/373).
- Allow changing the log verbosity (https://github.com/dic-iit/bipedal-locomotion-framework/pull/385)
- Implement the CoMZMP controller (https://github.com/dic-iit/bipedal-locomotion-framework/pull/387)

### Changed
- Add common Python files to gitignore (https://github.com/dic-iit/bipedal-locomotion-framework/pull/338)
- General improvements of `blf-calibration-delta-updater` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/361)
- Add the possibility to control a subset of coordinates in `IK::SE3Task` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/356)
- Add the possibility to control a subset of coordinates in `IK::CoMTask` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/357)
- Reduce the duplicate code in IK and TSID (https://github.com/dic-iit/bipedal-locomotion-framework/pull/364)
- `QPFixedBaseTSID` now inherits from `QPTSID` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/366)
- Enable the Current control in `RobotInterface` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/375)
- Add the possibility to disable and enable the PD controllers in `IK::SE3Task` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/373).
- Add the possibility to use manif objects in the ForwardEuler integrator (https://github.com/dic-iit/bipedal-locomotion-framework/pull/379).

### Fix
- Fixed the crashing of `YarpSensorBridge` while trying to access unconfigured control board sensors data by adding some checks (https://github.com/dic-iit/bipedal-locomotion-framework/pull/378)
- Fixed the compilation of Python bindings (enabled by the `FRAMEWORK_COMPILE_PYTHON_BINDINGS` CMake option) when compiling with Visual Studio (https://github.com/dic-iit/bipedal-locomotion-framework/pull/380).
- Fixed the `TOML` and `YARP` implementation of the parameters handler when a `std::vector<bool>` is passed to the `setParameter()` method (https://github.com/dic-iit/bipedal-locomotion-framework/pull/390).

## [0.2.0] - 2021-06-15
### Added
- Implement IRobotControl python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/200)
- Implement ISensorBridge python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/203)
- Implement `LeggedOdometry` class as a part of `FloatingBaseEstimators` library and handle arbitrary contacts in `FloatingBaseEstimator`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/151)
- Implement the possibility to set a desired reference trajectory in the TimeVaryingDCMPlanner. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/208)
- Implement SchmittTriggerDetector python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/213)
- Implement ModelComputationsHelper for quick construction of KinDynComputations object using parameters handler (https://github.com/dic-iit/bipedal-locomotion-framework/pull/216)
- Implement FloatingBaseEstimator and LeggedOdometry python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/218)
- Add spdlog as mandatory dependency of the project (https://github.com/dic-iit/bipedal-locomotion-framework/pull/225)
- Implement `ICameraBridge` and `IPointCloudBridge` interface classes as a part of `PerceptionInterface` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/165)
- Implement `RealSense` driver class as a part of `PerceptionCapture` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/165)
- Implement `realsense-test` utility application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/165)
- Implement the inverse kinematics component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/229)
- Implement LinearizedFrictionCone class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/244)
- Added a check on whether the installed public headers have the correct folder structure (https://github.com/dic-iit/bipedal-locomotion-framework/pull/247)
- Implement python bindings for VariablesHandler class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/234)
- Implement `PerceptionFeatures` library and implement `ArucoDetector`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/159)
- Implement FixedBaseDynamics class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/242)
- Implemented Sink and Source classes (https://github.com/dic-iit/bipedal-locomotion-framework/pull/267)
- Implement the IClock, StdClock and YarpClock classes (https://github.com/dic-iit/bipedal-locomotion-framework/pull/269)
- Implement `YarpCameraBridge` class for Yarp implementation of ICameraBridge (https://github.com/dic-iit/bipedal-locomotion-framework/pull/237)
- Implement `PointCloudProcessor` class and modify `realsense-test` to test point clouds handling with Realsense. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/236)
- Implement `AdvanceableRunner` and `SharedResource` classes in System component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/272)
- Implement `handleQuitSignals()` function in System component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/277)
- Implement TaskSpaceInverseDynamics interface (https://github.com/dic-iit/bipedal-locomotion-framework/pull/279)
- Implement `Wrench` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/279)
- Implement `SO3Task` in `TSID` component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/281)
- Implement clone method in ParametersHandler classes (https://github.com/dic-iit/bipedal-locomotion-framework/pull/288)
- Implement `VariablesHandler::clear()` and `VariablesHandler::initialize()` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/291)
- Implement the possibility to set the default contact in the `ContactList` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/297)
- Implement `FixedFootDetector` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/284)
- Implement QPFixedBaseTSID class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/251)
- Implement `YarpImplementation::setFromFile()` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/307)
- Implement `CoMTask` in TSID (https://github.com/dic-iit/bipedal-locomotion-framework/pull/304)
- Implement `YarpParametersHandler` bindings (https://github.com/dic-iit/bipedal-locomotion-framework/pull/309)
- Implement `contactListMapFromJson()` and `contactListMapToJson()` methods and python bindings (https://github.com/dic-iit/bipedal-locomotion-framework/issues/316)
- Implement a matioCpp-based strain2 sensors' FT-IMU logger example device (https://github.com/dic-iit/bipedal-locomotion-framework/pull/326)
- Implement `TomlImplementation` in `ParametersHandler` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/328)
- Implement blf_calibration_delta_updater.py application (https://github.com/dic-iit/bipedal-locomotion-framework/pull/332)

### Changed
- Move all the Contacts related classes in Contacts component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/204)
- Move all the ContactDetectors related classes in Contacts component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/209)
- The DCMPlanner and TimeVaryingDCMPlanner initialize functions take as input an std::weak_ptr. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/208)
- Use `Math::StandardAccelerationOfGravitation` instead of hardcoding 9.81. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/211)
- Convert iDynTree types in FloatingBaseEstimators component to Eigen/manif types (https://github.com/dic-iit/bipedal-locomotion-framework/pull/215)
- Use std::optional instead of raw pointer in ISensorBridge. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/226)
- Use `System::LinearTask` in TSID component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/240)
- Restructure python bindings in submodules (https://github.com/dic-iit/bipedal-locomotion-framework/pull/238)
- Integrators and DynamicalSystems are now in the `ContinuousDynamicalSystem` component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/242)
- Add Input template class to `System::Advanceable` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/267)
- Add support for landmarks and kinematics-free estimation in `FloatingBaseEstimators`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/254)
- If FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES is OFF, for Python bindings use installation directory provided by sysconfig Python module. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/274)
- Reduce memory allocation in `YarpSensorBridge` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/278)
- Use `TextLogging` in `VariablesHandler` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/291)
- Fix `YarpImplementation::setParameterPrivate()` when a boolean or a vector of boolean is passed (https://github.com/dic-iit/bipedal-locomotion-framework/pull/311)
- Add `foot_take_off_acceleration` and `foot_take_off_velocity` parameters in the `SwingFootPlanner` class (https://github.com/dic-iit/bipedal-locomotion-framework/issues/323)
- Change the parameters handler verbosity (https://github.com/dic-iit/bipedal-locomotion-framework/pull/330)
- Restore backward compatibility of SwingFootPlanner parameters (https://github.com/dic-iit/bipedal-locomotion-framework/pull/334)
- Bump manif version to 0.0.4  (https://github.com/dic-iit/bipedal-locomotion-framework/pull/339)

### Fixed
- Fix missing implementation of `YarpSensorBridge::getFailedSensorReads()`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/202)
- Fixed `mas-imu-test` configuration files after FW fix.
- Fixed the implementation ``YarpSensorBridge::attachAllSixAxisForceTorqueSensors()`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/231)
- Avoid the "Generating the Urdf Model from" message to appear when doing ccmake. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/243)
- Fixed the installation path of public headers related to perception libraries. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/245)
- Fixed InstallBasicPackageFiles to avoid the same problem of https://github.com/dic-iit/matio-cpp/pull/41 (https://github.com/dic-iit/bipedal-locomotion-framework/pull/253)
- Call `positionInterface->setRefSpeeds()` only once when a position reference is set in `YarpRobotControl` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/271)
- Fix initialization of reference frame for world in `LeggedOdometry` class. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/289)
- `LeggedOdometry::Impl::updateInternalContactStates()` is now called even if the legged odometry is not initialize. This was required to have a meaningful base estimation the first time `LeggedOdometry::changeFixedFrame()` is called. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/292)
- Avoid to use the default copy-constructor and copy-assignment operator in `ContactPhaseList` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/295)
- Fix `toString()` method of `VariablesHandler` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/302)
- Fix in `YarpUtilities::getVectorFromSearchable` when a vector of boolean is passed as input (https://github.com/dic-iit/bipedal-locomotion-framework/pull/313)
- Various fixes for the yarp devices (https://github.com/dic-iit/bipedal-locomotion-framework/pull/337)

## [0.1.1] - 2021-05-08
### Fix
- Fix the documentation in `TemplateHelpers.h`

## [0.1.0] - 2021-02-22
### Added
- The `CHANGELOG.md` file
- The `cmake/BipedalLocomotionControllersFindDepencies.cmake` file
- The `cmake/AddInstallRPATHSupport.cmake` file
- The `cmake/AddUninstallTarget.cmake` file
- The `cmake/FindEigen3.cmake` file
- The `cmake/InstallBasicPackageFiles.cmake` file
- Implement the first version of the `BipedalLocomotionControllers` interface
- Implement the  first version of the `YarpUtilities` library
- Implement `ParametersHandler` library (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/13)
- Implement `GenericContainer::Vector` (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/29)
- Implement `Estimators` library (https://github.com/dic-iit/bipedal-locomotion-controllers/pull/23)
- Implement `Contact` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/43 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/45)
- Implement the first version of the `TimeVaryingDCMPlanner` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/61)
- Implement the Quintic Spline class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/83)
- Implement the `ConvexHullHelper` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/51)
- Implement the `DynamicalSystem` and `Integrator` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/46)
- Implement the `IRobotControl` interface and the YARP specialization (https://github.com/dic-iit/bipedal-locomotion-framework/pull/97, https://github.com/dic-iit/bipedal-locomotion-framework/pull/192)
- Add `SensorBridge` interface (https://github.com/dic-iit/bipedal-locomotion-framework/pull/87)
- Add the `YarpSensorBridge` Implementation (https://github.com/dic-iit/bipedal-locomotion-framework/pull/106)
- Added `CommonConversions`, `ManifConversions`, and `matioCppConversions` libraries to handle type conversions. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/138 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/143)
- Implement the `JointPositionTracking` application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/136)
- Initial implementation of Python bindings using pybind11 (https://github.com/dic-iit/bipedal-locomotion-framework/pull/134)
- Implement `FloatingBaseEstimatorDevice` YARP device for wrapping floating base estimation algorithms. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/130)
- Implement Continuous algebraic Riccati equation function (https://github.com/dic-iit/bipedal-locomotion-framework/pull/157)
- Implement YARP based `ROSPublisher` in the `YarpUtilities` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/156)
- Implement example YARP device `ROSPublisherTestDevice` for understanding the usage of `ROSPublisher`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/160)
- Implement `TSID` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/167, https://github.com/dic-iit/bipedal-locomotion-framework/pull/170, https://github.com/dic-iit/bipedal-locomotion-framework/pull/178)
- Implement the `JointTrajectoryPlayer` application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/169)29ed234a1c
- Implement `ContactDetectors` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/142)
- Added `mas-imu-test` application to check the output of MAS IMUs (https://github.com/dic-iit/bipedal-locomotion-framework/pull/62)
- Implement motor currents reading in `YarpSensorBridge`. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/187)

[Unreleased]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.6.0...master
[0.6.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/ami-iit/bipedal-locomotion-framework/releases/tag/v0.1.0
