# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
### Added
- Implement `CubicSpline` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/344)
- Implement `PWM` control in RobotControl class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/346)
- Implement `ContactWrenchCone` class in Math component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/352)
- Implement `skew` function in Math component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/352)
- Implement `QPTSID` class (https://github.com/dic-iit/bipedal-locomotion-framework/pull/366)
- Implement motor pwm, motor encoders, wbd joint torque estimates, pid reading in `YarpSensorBridge`(https://github.com/dic-iit/bipedal-locomotion-framework/pull/359).
- Implement FeasibleContactWrenchTask for TSID component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/369).

### Changed
- Add common Python files to gitignore (https://github.com/dic-iit/bipedal-locomotion-framework/pull/338)
- General improvements of `blf-calibration-delta-updater` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/361)
- Add the possibility to control a subset of coordinates in `IK::SE3Task` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/356)
- Add the possibility to control a subset of coordinates in `IK::CoMTask` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/357)
- Reduce the duplicate code in IK and TSID (https://github.com/dic-iit/bipedal-locomotion-framework/pull/364)
- `QPFixedBaseTSID` now inherits from `QPTSID` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/366)

### Fix

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

[Unreleased]: https://github.com/dic-iit/bipedal-locomotion-framework/compare/v0.2.0...master
[0.2.0]: https://github.com/dic-iit/bipedal-locomotion-framework/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/dic-iit/bipedal-locomotion-framework/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/dic-iit/bipedal-locomotion-framework/releases/tag/v0.1.0
