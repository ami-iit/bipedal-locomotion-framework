# Changelog
All notable changes to this project are documented in this file.

## [unreleased]
### Added
- Added Vector Collection Server for publishing information for real-time users in the YARPRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/796)
- Set submodel states from IMUs in RDE and add friction torques as measurement (https://github.com/ami-iit/bipedal-locomotion-framework/pull/793)
- Add streaming of arm fts in YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/803)
- 🤖  [ `ergoCubGazeboV1_1`] Add configuration files to log data with the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/806, https://github.com/ami-iit/bipedal-locomotion-framework/pull/808)
- Added a unit test code for the `UnicyclePlanner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/815)
- Added a member function to set the gravity vector of the `CentroidalDynamics` Continuous Dynamical System (https://github.com/ami-iit/bipedal-locomotion-framework/pull/821).
- Add the possibility to set the exogenous signal for the `IK::CoMTask` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/824)
- Add getJointTorques to python binding in SensorBridge (https://github.com/ami-iit/bipedal-locomotion-framework/pull/825)
- Add `System::TimeProfiler` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/826)
- Add the possibility to programmatically build a `QPTSID` problem from the content of a `ParametersHandler` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/828)
- Add support for testing if a portion of code allocates memory via `MemoryAllocationMonitor` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/768)
- Implement `Math::ZeroOrderSpline` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/834)
- Add the possibility to get only position or position/velocity from the spline (https://github.com/ami-iit/bipedal-locomotion-framework/pull/834)
- Add the possibility to set the number of threads used by onnxruntime in `MANN` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/836)
- Implement `ButterworthLowPassFilter` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/838)
- Implement `Conversions::toiDynTreeRot` function (https://github.com/ami-iit/bipedal-locomotion-framework/pull/842)
- Add the `Planners::UnicycleTrajectoryGenerator` to mimic the functionalities of the unicycle planner deployed in [walking-controllers](https://github.com/robotology/walking-controllers) (https://github.com/ami-iit/bipedal-locomotion-framework/pull/845)
- Create python bindings of `VectorsCollection` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/854)
- Added a simple motor control example (https://github.com/ami-iit/bipedal-locomotion-framework/pull/855)
- Add `setControlModeAsync` function to set motor control mode in an asynchronous process (https://github.com/ami-iit/bipedal-locomotion-framework/pull/860)
- Add launch parameter to `blf-logger-with-audio.sh` script to set logger launch file (https://github.com/ami-iit/bipedal-locomotion-framework/pull/867)
- Add `getJointLimits` function to set get actuated joints position limits (https://github.com/ami-iit/bipedal-locomotion-framework/pull/868)
- Add the possibility to disable streaming of joint encoder acceleration measurements (https://github.com/ami-iit/bipedal-locomotion-framework/pull/876)

### Changed
- 🤖 [ergoCubSN001] Add logging of the wrist and fix the name of the waist imu (https://github.com/ami-iit/bipedal-locomotion-framework/pull/810)
- Export the CoM velocity and the angular momentum trajectory in the `CentroidalMPC` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/818)
- Require `iDynTree v10.0.0` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/832)
- Refactor `YarpRobotControl::setReferences` function to include optional current joint values and avoid to switch control mode in `YarpRobotControl::setReferences` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/833)
- Set the gravity vector as an input argument of the `CentroidalMPC` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/823)
- Refactor the `Planners::UnicyclePlanner` to mimic the functionalitites of the planner deployed in [walking-controllers](https://github.com/robotology/walking-controllers) (https://github.com/ami-iit/bipedal-locomotion-framework/pull/844)

### Fixed
- Fix the barrier logic for threads synchronization (https://github.com/ami-iit/bipedal-locomotion-framework/pull/811)
- InstallBasicPackageFiles: Fix bug of OVERRIDE_MODULE_PATH that corrupt `CMAKE_MODULE_PATH` values set by blf transitive dependencies (https://github.com/ami-iit/bipedal-locomotion-framework/pull/827)
- InstallBasicPackageFiles: Fix compatibility with CMake 3.29.1 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/835)
- Fix `YARPRobotLoggerDevice` excessively long time horizon for signals logged with `YARP_CLOCK` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/839)
- Fix crash in `robot_control.set_references()` if the passed vector is not the correct size (https://github.com/ami-iit/bipedal-locomotion-framework/pull/852)
- Fix `VectorsCollectionClient.read_data(False)` to provide `collection` as output and avoid segmentation fault (https://github.com/ami-iit/bipedal-locomotion-framework/pull/850)
- Fix `YARPRobotLoggerDevice` logging of vectors and text (https://github.com/ami-iit/bipedal-locomotion-framework/pull/869)

### Removed

## [0.18.0] - 2024-01-23
### Changed
- 🤖 [`ergoCubSN001`] Update `YarpRobotLoggerDevice` configuration file to exclude the head-imu and include the arms FTs  (https://github.com/ami-iit/bipedal-locomotion-framework/pull/798)
- Avoid to call `BufferedPort::prepare` every time `VectorsCollectionServer::populateData` is called (https://github.com/ami-iit/bipedal-locomotion-framework/pull/790)
- Check that the size the gains vectors in the `IK::JointTrackingTask`, `IK::JointLimitsTask` and `TSID::JointLimitsTask` are correct (https://github.com/ami-iit/bipedal-locomotion-framework/pull/797)
- Check that the size of the weights in `MultiStateWeightProvider` are all the same (https://github.com/ami-iit/bipedal-locomotion-framework/pull/797)

## [0.17.0] - 2023-12-23
### Added
- Implement `ContactList::getNextContact` and `ContactList::getActiveContact` in `Contact` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/764)
- Implement `MANN::generateDummyMANNOutput` and `MANN::generateDummyMANNInput` in `ML` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/771)
- Add `MANNAutoregressive` and `MANNTrajectoryGenerator` examples (https://github.com/ami-iit/bipedal-locomotion-framework/pull/771)
- Implement `Spline::evaluateOrderedPoints` to evaluate the spline at a set of time ordered points (https://github.com/ami-iit/bipedal-locomotion-framework/pull/773)
- Add process model for external contacts in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/759)
- 🤖 Add `ergoCubSN001` configuration files for the `balancing-position-control` application (https://github.com/ami-iit/bipedal-locomotion-framework/pull/776)
- Implement `VectorsCollectionServer` python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/776)
- Implement `toString()` methods in `PlannedContact`, `ContactList`, `ContactPhase` and `ContactPhaseList` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/777)
- Implement `LinearSpline` in `Math` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/782)
- Add the support of QP problems with no constraint in `QPInverseKinematics` and `QPTSID` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/784)
- Implement `blf-joints-grid-position-tracking` application in `utilities` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/787)
- Add the possibility to resample the contact in a given contact list (https://github.com/ami-iit/bipedal-locomotion-framework/pull/788)


### Changed
- Restructure of the `CentroidalMPC` class in `ReducedModelControllers` component. Specifically, the `CentroidalMPC` now provides a contact phase list instead of indicating the next active contact. Additionally, users can now switch between `ipopt` and `sqpmethod` to solve the optimization problem. Furthermore, the update allows for setting the warm-start for the non-linear solver. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/766)
- Restructured the swing foot planner to handle corners case that came out while testing DNN-MPC integration (https://github.com/ami-iit/bipedal-locomotion-framework/pull/765)
- Avoid returning internal references for `get_next_contact` `get_present_contact` and `get_active_contact` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/765)
- Introducing Metadata Support for `VectorsCollection` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/767)
- Restructure `MANNAutoregressive` to effectively manage resets (https://github.com/ami-iit/bipedal-locomotion-framework/pull/771)
- Restructure `MANNTrajectoryGenerator` to remove the need for foot positions in `setInitialState` and enhanced reset capabilities. Corrected position and time scaling for precise CoM, base, and feet positioning (https://github.com/ami-iit/bipedal-locomotion-framework/pull/771)
- Move `Spline` into `Math` component. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/773)
- Deprecate `Planners::Spline`, `Planners::QuinticSpline`, `Planners::CubicSpline` in favor of `Math::Spline`, `Math::QuinticSpline`, `Math::CubicSpline` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/773)
- Change use of dynamics name for fts, contacts, accelerometers, gyroscopes in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/759)
- 🤖 Modify the `YarpRobotLoggerDevice` of `ergoCubSN001` to be compliant with the latest [robots-configuration files](https://github.com/robotology/robots-configuration/pull/598) (https://github.com/ami-iit/bipedal-locomotion-framework/pull/772)
- Modify the `balancing-position-control` application to be compliant with the new implementation of the spline and the `VectorsCollectionServer` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/776)
- Add the joint tracking error plot in joint-position-tracking script (https://github.com/ami-iit/bipedal-locomotion-framework/pull/775/)
- Force the MPC to require correctly time-sampled contacts (https://github.com/ami-iit/bipedal-locomotion-framework/pull/788)

### Removed
- Remove the latex dependency in the joint-position-tracking script (https://github.com/ami-iit/bipedal-locomotion-framework/pull/775/)

## [0.16.1] - 2023-11-17
### Changed
- [ergoCubSN000] Update the YarpRobotLogger configuration file to be mpliant with robots-configuration v2.5.2 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/763)

### Fixed
- Fixed compilation on Windows of fmt formatter for Eigen types (https://github.com/ami-iit/bipedal-locomotion-framework/pull/762)

## [0.16.0] - 2023-11-15
### Added
- Add the possibility to control a subset of coordinates in `TSID::CoMTask` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/724, https://github.com/ami-iit/bipedal-locomotion-framework/pull/727)
- Add the possibility to set the maximum number of accepted deadline miss in `System::AdvanceableRunner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/726)
- Add the `getControllerOutput` method to the `TSID::SE3Task` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/740)
- Implement the python bindings for the `Wrench` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/716)
- Implement the python bindings for the `SimplifiedModelControllers` components (https://github.com/ami-iit/bipedal-locomotion-framework/pull/716)
- 🤖 Add the configuration files to use `YarpRobotLoggerDevice` with `ergoCubSN001` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/747)
- Added the ``DistanceTask`` and ``GravityTask`` to the IK (https://github.com/ami-iit/bipedal-locomotion-framework/pull/717)
- Add the possibility to control a subset of linear coordinates in `TSID::SE3Task` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/738)
- Implement `GlobalCoPEvaluator` in `Contacts` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/745)
- Implement `Wrench::getLocalCoP()` method (https://github.com/ami-iit/bipedal-locomotion-framework/pull/745)
- Add tests for classes of `RobotDynamicsEstimator` library (https://github.com/ami-iit/bipedal-locomotion-framework/pull/743)
- Implement inequality operator for the `PlannedContact` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/750)
- Finalize `RobotDynamicsEstimator` library and add complete library test (https://github.com/ami-iit/bipedal-locomotion-framework/pull/744)
- Add Python bindings for `RobotDynamicsEstimator` library (https://github.com/ami-iit/bipedal-locomotion-framework/pull/755)
- Add possibility to set the regularization on the mass matrix for the `TSID::JointDynamicsTask` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/722)
- Implement `RobotDynamicsEstimatorDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/756)

### Changed
- Remove the possibility to disable the telemetry in `System::AdvanceableRunner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/726)
- Change implementation of classes used in `RobotDynamicsEstimator` to optimize performance (https://github.com/ami-iit/bipedal-locomotion-framework/pull/731)
- CMake: Permit to explictly specify Python installation directory by setting the `FRAMEWORK_PYTHON_INSTALL_DIR` CMake variable (https://github.com/ami-iit/bipedal-locomotion-framework/pull/741)
- Remove outdated tests for `RobotDynamicsEstimator` library (https://github.com/ami-iit/bipedal-locomotion-framework/pull/742)
- Modify CI to install `RobotDynamicsEstimator` library (https://github.com/ami-iit/bipedal-locomotion-framework/pull/746)
- Restructure the balancing-position-control script (https://github.com/ami-iit/bipedal-locomotion-framework/pull/716)
- Avoid to download the robot and the network models if the tests are not enabled in `ML` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/749)
- Update the CMakeLists.txt to be compliant with `Python 3.12` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/752)

### Fixed
- Fix timestamp logging for the cameras (https://github.com/ami-iit/bipedal-locomotion-framework/pull/748)
- Fix missing include(FetchContent) in Python bindings CMake code (https://github.com/ami-iit/bipedal-locomotion-framework/pull/757)

## [0.15.0] - 2023-09-05
### Added
- 🤖 Add the configuration files to use `YarpRobotLogger` with  `ergoCubGazeboV1` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/690)
- Implement `RK4` integrator in `ContinuousDynamicalSystem` component and expose the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/711)
- Implement `blf-balancing-torque-control` in `utilities` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/707)
- Implement `setControlMode` in `IRobotControl` and the associated python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/707)
- Expose `ContactBase` methods for `DiscreteGeometryContact` bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/712)
- Expose the CoM trajectory and angular momentum trajectory in `MANNTrajectoryGenerator` bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/712)
- Expose the CoM trajectory computed by the `CentroidalMPC` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/712)
- Implement python bindings for `CameraBridge` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/695)
- Implement `constructRDGBSensorClient()` in `RobotInterface` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/695)

### Changed
- Use `std::chorno::nanoseconds` in `clock` and `AdvanceableRunner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/702)
- Give the possibility to set an external wrench in the `CentroidalDynamics` instead of a pure force (https://github.com/ami-iit/bipedal-locomotion-framework/pull/705)
- Use c version of `qhull` in the Planner component. This fixes the compatibility with PCL in ubuntu 20.04 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/713)
- Make `ICameraBridge::isValid()` virtual function (https://github.com/ami-iit/bipedal-locomotion-framework/pull/695)
- icub-models 2.0.0 changed the name of the FT sensors in the iCub's URDF from being named `<identifier>_ft_sensor` (like `l_arm_ft_sensor`, `l_leg_ft_sensor`, ...) to `<identifier>_ft` (like `l_arm_ft`, `l_leg_ft`, ...). However, the yarprobotinterface configuration files in blf continued to refer to the sensors as `<identifier>_ft_sensor`, creating errors for software that was trying to match sensors find in URDF and sensors as exposed by the YARP's multipleanalogsensorsserver device. This PR changes all the instances of FT sensor names in iCub-related configuration files contained in blf to `<identifier>_ft`, restoring compatibility with icub-models 2.0.0, robots-configuration releases >= 2.5.0 and ergocub-software >= 0.3.4, see https://github.com/robotology/robots-configuration/pull/562 for more details (https://github.com/ami-iit/bipedal-locomotion-framework/pull/720)


### Fixed
- Remove duplicated `find_package` in `BipedalLocomotionFrameworkDependencies.cmake` file (https://github.com/ami-iit/bipedal-locomotion-framework/pull/709)
- Fix handling of feedforward acceleration in `BipedalLocomotion::TSID::JointTrackingTask::setSetPoint` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/719)

## [0.14.1] - 2023-07-07
### Fixed
- Fix python bindings compilation when toml is not installed (https://github.com/ami-iit/bipedal-locomotion-framework/pull/700)
- Fix `YarpRobotLoggerDevice` if `YARP_ROBOT_NAME` is not defined (https://github.com/ami-iit/bipedal-locomotion-framework/pull/701)

## [0.14.0] - 2023-07-04
### Added
- Implement a class to perform inference with the [MANN network](https://github.com/ami-iit/mann-pytorch) (https://github.com/ami-iit/bipedal-locomotion-framework/pull/652, https://github.com/ami-iit/bipedal-locomotion-framework/pull/686)
- Add some useful methods to the `SubModel` and `SubModelKinDynWrapper` classes (https://github.com/ami-iit/bipedal-locomotion-framework/pull/661)
- Implement `Dynamics`, `ZeroVelocityDynamics`, and `JointVelocityStateDynamics` classes in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/662)
- Implement `AccelerometerMeasurementDynamics` and `GyroscopeMeasurementDynamics` classes in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/666)
- Implement `FrictionTorqueStateDynamics` and `MotorCurrentMeasurementDynamics` classes in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/667)
- Add logging of FTs on the ergoCub legs and fix FT and IMU port names (https://github.com/ami-iit/bipedal-locomotion-framework/pull/671)
- Add the possibility to log the video stream as set of frames or as video in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/672)
- Add the possibility to log the RGBD stream in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/672)
- Implement `LieGroupDynamics` in `ContinuousDynamicalSystem` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/659)
- Implement `Conversions::toiDynTreePose()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/659)
- Implement `Conversions::toiDynTreePose()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/659)
- Implement `MANNAutoregressive` class in `ML` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/659)
- Implement`UkfState` class in `RobotDynamicsEstimator` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/669)
- Implement `MANNTrajectoryGenerator` class in `ML` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/668)
- Implement a bash script to automatically log the audio along with the YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/681)
- Implement `QuadraticBezierCurve` in `Math` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/689)
- Implement `MANNAutoregressiveInputBuilder` in `ML` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/689)
- Implement the `CentroidalMPC` in `ReducedModelControllers`component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/645)
- Implement the `BaseEstimatorFromFootIMU` in the `Estimators` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/641)
- Add bindings for `CentroidalMPC`, `CentroidalDynamics`, `ContactPhaseList`, `DiscreteGeometryContact` and `Corner` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/650)

### Changed
- Restructure application folders of `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/670)
- 🤖 [ergoCubSN000] Clean the mas remapper files of the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/673)
- 🤖 [ergoCubSN000] Enable the logging of the realsense camera `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/672)
- Add the possibility to force the internal state of the `SchmittTrigger` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/684)
- Add the possibility to update the contact list in the swing foot planner when the contact is not active and the new orientation is different from the previous one (https://github.com/ami-iit/bipedal-locomotion-framework/pull/688)
- Add the possibility to set the boundary condition for the velocity and acceleration of the `SO3Planner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/688)
- Satisfies the rule of 5 for `InputPort`, `OutputPort`, `Advanceable`, `Sink` and `Source` in `System` component (https://github.com/dic-iit/bipedal-locomotion-framework/pull/397).

### Fixed
- Fix RobotDynamicsEstimator compilation dependencies (https://github.com/ami-iit/bipedal-locomotion-framework/pull/665)
- Fix the metadata assignment of the rgbd cameras (https://github.com/ami-iit/bipedal-locomotion-framework/pull/672)
- Fix the RGB to BGR conversion in YarpCameraBridge for FlexImage (https://github.com/ami-iit/bipedal-locomotion-framework/pull/672)
- Make the YarpSensorBridge compatible with MAS that use the same name between gyro ac magn and
  orientation (https://github.com/ami-iit/bipedal-locomotion-framework/pull/692)

## [0.13.0] - 2023-04-22
### Added
- Implement the `DiscreteGeometryContact` in Contacts component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/626)
- Implement the `SchmittTrigger` in component `Math` and the associated python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/624)
- Add the support of `std::chrono` in The text logging (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Add the possibility to retrieve and set duration from the `IParametersHandler` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Add the possibility to update the contact list in the swing foot trajectory planner (https://github.com/ami-iit/bipedal-locomotion-framework/pull/637)
- Implement `System::NamedTuple` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/642)
- Implement `ContinuousDynamicalSystem::CentroidalDynamics` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/642)

### Changed
- Update the `IK tutorial` to use `QPInverseKinematics::build` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/621)
- Handle case where no FT sensors are specified to split the model (https://github.com/ami-iit/bipedal-locomotion-framework/pull/625)
- General restructure of the `ContactDetector`and the derived classes (`SchmittTriggerDetector` and `FixedFootDetector`) (https://github.com/ami-iit/bipedal-locomotion-framework/pull/624)
  Thanks to this refactory the `FixedFootDetector` usage becomes similar to the others `advanceable`.
  Indeed now `FixedFootDetector::advace()` considers the input set by the user and provides the corresponding output.
  ⚠️  Even if this modification do not break the API the user may notice some strange behavior if `advance` was called after getting the output of the detector.
- Restructure the `Contacts` component to handle time with `std::chrono::nanoseconds` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Restructure the `Planners` component to handle time with `std::chrono::nanoseconds` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Restructure the `FloatingBaseEstimator` component to handle time with `std::chrono::nanoseconds`(https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Update the `blf-position-tracking` to handle time with `std::chrono::nanoseconds` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Update the python bindings to consider the time with `std::chrono::nanoseconds` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/630)
- Robustify SubModelCreator and SubModelKinDynWrapper tests (https://github.com/ami-iit/bipedal-locomotion-framework/pull/631)
- `SwingFootTrajectoryPlanner::advance()` must be called before getting the output (https://github.com/ami-iit/bipedal-locomotion-framework/pull/637)
- Update the already existing classes in `ContinuousDynamicalSystem`to be compatible with the `System::NamedTuple` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/642)
- Update the code to be compatible with LieGroupControllers v0.2.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/653)
- Allow use of vectors for task gains (https://github.com/ami-iit/bipedal-locomotion-framework/pull/654)
- `Catch2` is now downloaded with `FetchContent` if `BUILD_TESTING` is set to `ON` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/655)
- The tests now uses`Catch2 v3` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/655)
- Add the possibility to set vectorial gains in CoM IK and TSID tasks (https://github.com/ami-iit/bipedal-locomotion-framework/pull/656)

### Fixed
- Return an error if an invalid `KinDynComputations` object is passed to `QPInverseKinematics::build()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/622)
- Fix `QPTSID` documentation (https://github.com/ami-iit/bipedal-locomotion-framework/pull/634)
- Fix error messages in `QPTSID` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/639)
- Fix compilation failure when using CMake 3.26.1 and pybind11 2.4.3 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/643)
- Fixed changelog checker (https://github.com/ami-iit/bipedal-locomotion-framework/pull/856)

## [0.12.0] - 2023-03-07
### Added
- Add the possibility to attach all the multiple analog sensor clients  (https://github.com/ami-iit/bipedal-locomotion-framework/pull/569)
- Add a tutorial for the inverse kinematics  (https://github.com/ami-iit/bipedal-locomotion-framework/pull/596)
- Implement the ROS2 sink for the `TextLogging` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/587)
- Implement the `QPFixedBaseInverseKinematics` in the `IK` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/599)
- 🤖 [ergoCubSN000] Add configuration files for the YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/600)
- Add functions to split a model in a set of submodels in the Estimator component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/604)
- Add the possibity to call the advanceable capabilities of the `QuinticSpline` from the python (https://github.com/ami-iit/bipedal-locomotion-framework/pull/609)
- Implement the `CubicSpline` python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/609)
- Implement the python bindings for the iDynTree to manif conversions (https://github.com/ami-iit/bipedal-locomotion-framework/pull/610)
- Implement `blf-balancing-position-control` application (https://github.com/ami-iit/bipedal-locomotion-framework/pull/611)
- Implement the python bindings for `YarpTextLogging` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/611)
- Add SubModelKinDynWrapper class to handle the `KinDynComputation` object of a sub-model (https://github.com/ami-iit/bipedal-locomotion-framework/pull/605)
- Implement the `JointLimitsTask` for the IK (https://github.com/ami-iit/bipedal-locomotion-framework/pull/603)
- Add the possibility to programmatically build an IK problem from a configuration file (https://github.com/ami-iit/bipedal-locomotion-framework/pull/614, https://github.com/ami-iit/bipedal-locomotion-framework/pull/619)

### Changed
- Ask for `toml++ v3.0.1` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/581)
- The YarpRobotLogger will now automatically connect to the exogenous signal port if available (https://github.com/ami-iit/bipedal-locomotion-framework/pull/570/)
- 🤖 [iCubGenova09] Add the left and right hands skin (raw and filtered) data acquisition (https://github.com/ami-iit/bipedal-locomotion-framework/pull/570/)
- Add informative prints `YarpSensorBridge::Impl` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/569)
- The minimum version of iDynTree now supported is iDynTree 4.3.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/588).
- Allow using the `iDynTree swig` bindings in `QPFixedBaseTSID` for the kindyncomputation object (https://github.com/ami-iit/bipedal-locomotion-framework/pull/599)
- Add the possibility to customize the video codec in the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/607)

### Fix
- Return an invalid `PolyDriverDescriptor` if `description` is not found in `constructMultipleAnalogSensorsRemapper()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/569)
- Fix compatibility with OpenCV 4.7.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/589)
- Fix `attachRemappedRemoteControlBoard` in `YarpSensorBridge` when the `RemoteControlBoard` is not the first polydriver in the polydriverlist (https://github.com/ami-iit/bipedal-locomotion-framework/pull/608)
- Fix race condition in System::ClockBuilder (https://github.com/ami-iit/bipedal-locomotion-framework/pull/618)

## [0.11.1] - 2022-12-19
### Fix
- Fix the compilation of the `YarpRobotLoggerDevice` in `Windows` and `macOS` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/580)

## [0.11.0] - 2022-12-17
### Added
- Log the status of the system in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/571)
- Add the `ROS2` implementation for Clock class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/575)

### Changed
- YARP devices are now enabled by default if YARP is found (https://github.com/ami-iit/bipedal-locomotion-framework/pull/576).
- Restructure the python bindings to support _official_ `iDynTree` bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/578)
- Remove _unofficial_ `iDynTree` bindings based on pybind11  (https://github.com/ami-iit/bipedal-locomotion-framework/pull/578)

### Fix
- Fix compatibility with YARP 3.8 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/577).

## [0.10.0] - 2022-09-23
### Added
- Add the possibility to set the exogenous feedback for the `IK::SE3Task` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/567)
- Implement `RobotInterface::constructMultipleAnalogSensorsClient()` and `RobotInterface::constructMultipleAnalogsensorsRemapper()` methods (https://github.com/ami-iit/bipedal-locomotion-framework/pull/568)

### Changed
- Add the possibility to log only a subset of text logging ports in `YarpRobotLogger` device (https://github.com/ami-iit/bipedal-locomotion-framework/pull/561)
- Accept boolean as integer while getting an element from searchable in `YarpUtilities` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/566)

### Fix
- Fix typo in the `RobotInterface::constructGenericSensorClient()` documentation (https://github.com/ami-iit/bipedal-locomotion-framework/pull/568)
- Fix compatibility with qhull installed by vcpkg `2022.07.25` and robotology-superbuild-dependencies-vcpkg `0.10.1` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/565).

## [0.9.0] - 2022-09-09
### Added
- Implement the `MultiStateWeightProvider` in `ContinuousDynamicalSystem` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/555)
- Implement `PortInput` and `PortOutput`in `System` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/555)
- Implement `toManifTwist` in `Conversions` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/557)
- Add the default value for the desired spatial and angular velocity to the `IK::SO3Task` and `IK::SE3Task` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/557)
- Implement the `IK::R3Task` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/559)

### Changed
- Now `Advanceable` inherits from `PortInput` and `PortOutput` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/555)

### Fix
- Fix the dependency required to compile the YarpRobotLogger device (https://github.com/ami-iit/bipedal-locomotion-framework/pull/554)
- Fix the compatibility with fmt v9.0.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/556)

## [0.8.0] - 2022-07-29
### Added
- Add the possibility to log the YarpTextLogging in the YarpRobotLogger (https://github.com/ami-iit/bipedal-locomotion-framework/pull/541)
- Enable the logging FTs and IMU logging of iCubGenova09 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/546/)
### Changed
- Ported `YarpLoggerDevice` to `robometry`(https://github.com/ami-iit/bipedal-locomotion-framework/pull/533)
- bipedal locomotion framework now depends on YARP 3.7.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/541)
### Fix
- Avoid to use deprecated function cv::aruco::drawAxis in ArucoDetector to fix compilation with OpenCV 4.6.0 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/552)

## [0.7.0] - 2022-06-21
### Added
- Implement the python bindings for the clock machinery and for the yarp clock (https://github.com/ami-iit/bipedal-locomotion-framework/pull/500)
- Implement the `IWeightProvider` interface and the `ConstantWeightProvider` class in the System component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/506)
- Add installation of pip metadata files for when blf python bindings are installed only via CMake (https://github.com/ami-iit/bipedal-locomotion-framework/pull/508)
- Implement the python bindings for the VectorsCollection message and the associated buffered port (https://github.com/ami-iit/bipedal-locomotion-framework/pull/511)
- Implement the `VectorsCollectionWrapper` device for collection of arbitrary vector ports (https://github.com/ami-iit/bipedal-locomotion-framework/pull/512)
- Add reading of right upper leg FT for `iCubGenova09` and associated cartesian wrench in `YarpRobotLoggerDevice` configuration files (https://github.com/ami-iit/bipedal-locomotion-framework/pull/513)
- Add reading of right and left arms FT for `iCubGenova09` in `YarpRobotLoggerDevice` configuration files (https://github.com/ami-iit/bipedal-locomotion-framework/pull/515)
- Add reading of right and left arms and right upper leg FTs and cartesian wrenches for `iCubGazeboV3` in `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/525)
- Add the possibility to retrieve the temperature sensor from `SensorBridge` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/519)
- Add the possibility to set only the velocity in `CubicSpline::setInitialConditions` and `CubicSpline::setFinalConditions` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/528)
- Implement the python bindings for the `ContinuousDynamicalSystem` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/532)
- Add the possibility to log the video in the `YarpRobotLoggerDevice` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/516)

### Changed
- An error it will be returned if the user tries to change the clock type once the `clock()` has been already called once (https://github.com/ami-iit/bipedal-locomotion-framework/pull/500)
- Log the arms external wrenches on the YarpRobotLogger for iCubGenova09 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/502)
- IK and TSID now uses the weight provider to specify the weight associated to a task (https://github.com/ami-iit/bipedal-locomotion-framework/pull/506)
- The `Planners`, `System`, `RobotInterface` and `YarpImplementation` components are no more mandatory to compile the python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/511)
- Reorganize the multiple FT sensor and external wrench files into a single file in the YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/525)
- Save the robot name and the names of the channel's elements in the YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/522)
- Use icub-models to get the urdf models for the tests (https://github.com/ami-iit/bipedal-locomotion-framework/pull/526)
- The FT sensor are now considered as `multianalogsensor` in `YarpSensorBridge` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/519)
- Make `YarpRobotLogger` compatible with `yarp-telemetry` v0.5.1 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/535)
- Set for `yarp-telemetry` minimum version to v0.5.1 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/535)
- Make `YarpCameraBridge::getColorImage()` and `YarpCameraBridge::getDepthImage()` thread safe (https://github.com/ami-iit/bipedal-locomotion-framework/pull/516)
- Deprecate `YarpCameraBridge::get()` in favor of `YarpCameraBridge::getMetaData()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/516)
- Move from LGPL to BSD3 license (https://github.com/ami-iit/bipedal-locomotion-framework/pull/550)

### Fix
- Remove outdated includes in YarpRobotLoggerDevice.cpp (https://github.com/ami-iit/bipedal-locomotion-framework/pull/502)

## [0.6.0] - 2022-01-10
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
- Implement `AdvanceableRunner::isRunning()` method (https://github.com/ami-iit/bipedal-locomotion-framework/pull/395)
- Implement `ContactPhaseList::getPresentPhase()` method (https://github.com/ami-iit/bipedal-locomotion-framework/pull/396)
- Add a synchronization mechanism for the `AdvanceableRunner` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/403)
- Add the possibility to use spdlog with YARP (https://github.com/ami-iit/bipedal-locomotion-framework/pull/408)
- Add new Advanceable exposing `UnicyclePlanner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/320)

### Changed
- Add `name` parameter to the `AdvanceableRunner` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/406)
- Set the required `spdlog` version in the cmake file (https://github.com/ami-iit/bipedal-locomotion-framework/pull/415)
- Add features to FTIMULoggerDevice and rename it in YarpRobotLoggerDevice (https://github.com/ami-iit/bipedal-locomotion-framework/pull/405)

### Fix
- Fix missing components dependencies in the `CMake` machinery (https://github.com/ami-iit/bipedal-locomotion-framework/pull/414)
- Fixed missing include in `FloatingBaseEstimatorIO.h` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/417)

## [0.3.0] - 2021-08-12
### Added
- Implement `CubicSpline` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/344)
- Implement `PWM` control in RobotControl class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/346)
- Implement `ContactWrenchCone` class in Math component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/352)
- Implement `skew` function in Math component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/352)
- Implement `QPTSID` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/366)
- Implement motor pwm, motor encoders, wbd joint torque estimates, pid reading in `YarpSensorBridge`(https://github.com/ami-iit/bipedal-locomotion-framework/pull/359).
- Implement FeasibleContactWrenchTask for TSID component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/369).
- Implement python bindings for QPInverseKinematics class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/303)
- Implement `ControlTask` in for System component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/373).
- Allow changing the log verbosity (https://github.com/ami-iit/bipedal-locomotion-framework/pull/385)
- Implement the CoMZMP controller (https://github.com/ami-iit/bipedal-locomotion-framework/pull/387)

### Changed
- Add common Python files to gitignore (https://github.com/ami-iit/bipedal-locomotion-framework/pull/338)
- General improvements of `blf-calibration-delta-updater` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/361)
- Add the possibility to control a subset of coordinates in `IK::SE3Task` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/356)
- Add the possibility to control a subset of coordinates in `IK::CoMTask` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/357)
- Reduce the duplicate code in IK and TSID (https://github.com/ami-iit/bipedal-locomotion-framework/pull/364)
- `QPFixedBaseTSID` now inherits from `QPTSID` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/366)
- Enable the Current control in `RobotInterface` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/375)
- Add the possibility to disable and enable the PD controllers in `IK::SE3Task` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/373).
- Add the possibility to use manif objects in the ForwardEuler integrator (https://github.com/ami-iit/bipedal-locomotion-framework/pull/379).

### Fix
- Fixed the crashing of `YarpSensorBridge` while trying to access unconfigured control board sensors data by adding some checks (https://github.com/ami-iit/bipedal-locomotion-framework/pull/378)
- Fixed the compilation of Python bindings (enabled by the `FRAMEWORK_COMPILE_PYTHON_BINDINGS` CMake option) when compiling with Visual Studio (https://github.com/ami-iit/bipedal-locomotion-framework/pull/380).
- Fixed the `TOML` and `YARP` implementation of the parameters handler when a `std::vector<bool>` is passed to the `setParameter()` method (https://github.com/ami-iit/bipedal-locomotion-framework/pull/390).

## [0.2.0] - 2021-06-15
### Added
- Implement IRobotControl python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/200)
- Implement ISensorBridge python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/203)
- Implement `LeggedOdometry` class as a part of `FloatingBaseEstimators` library and handle arbitrary contacts in `FloatingBaseEstimator`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/151)
- Implement the possibility to set a desired reference trajectory in the TimeVaryingDCMPlanner. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/208)
- Implement SchmittTriggerDetector python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/213)
- Implement ModelComputationsHelper for quick construction of KinDynComputations object using parameters handler (https://github.com/ami-iit/bipedal-locomotion-framework/pull/216)
- Implement FloatingBaseEstimator and LeggedOdometry python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/218)
- Add spdlog as mandatory dependency of the project (https://github.com/ami-iit/bipedal-locomotion-framework/pull/225)
- Implement `ICameraBridge` and `IPointCloudBridge` interface classes as a part of `PerceptionInterface` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/165)
- Implement `RealSense` driver class as a part of `PerceptionCapture` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/165)
- Implement `realsense-test` utility application. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/165)
- Implement the inverse kinematics component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/229)
- Implement LinearizedFrictionCone class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/244)
- Added a check on whether the installed public headers have the correct folder structure (https://github.com/ami-iit/bipedal-locomotion-framework/pull/247)
- Implement python bindings for VariablesHandler class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/234)
- Implement `PerceptionFeatures` library and implement `ArucoDetector`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/159)
- Implement FixedBaseDynamics class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/242)
- Implemented Sink and Source classes (https://github.com/ami-iit/bipedal-locomotion-framework/pull/267)
- Implement the IClock, StdClock and YarpClock classes (https://github.com/ami-iit/bipedal-locomotion-framework/pull/269)
- Implement `YarpCameraBridge` class for Yarp implementation of ICameraBridge (https://github.com/ami-iit/bipedal-locomotion-framework/pull/237)
- Implement `PointCloudProcessor` class and modify `realsense-test` to test point clouds handling with Realsense. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/236)
- Implement `AdvanceableRunner` and `SharedResource` classes in System component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/272)
- Implement `handleQuitSignals()` function in System component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/277)
- Implement TaskSpaceInverseDynamics interface (https://github.com/ami-iit/bipedal-locomotion-framework/pull/279)
- Implement `Wrench` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/279)
- Implement `SO3Task` in `TSID` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/281)
- Implement clone method in ParametersHandler classes (https://github.com/ami-iit/bipedal-locomotion-framework/pull/288)
- Implement `VariablesHandler::clear()` and `VariablesHandler::initialize()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/291)
- Implement the possibility to set the default contact in the `ContactList` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/297)
- Implement `FixedFootDetector` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/284)
- Implement QPFixedBaseTSID class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/251)
- Implement `YarpImplementation::setFromFile()` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/307)
- Implement `CoMTask` in TSID (https://github.com/ami-iit/bipedal-locomotion-framework/pull/304)
- Implement `YarpParametersHandler` bindings (https://github.com/ami-iit/bipedal-locomotion-framework/pull/309)
- Implement `contactListMapFromJson()` and `contactListMapToJson()` methods and python bindings (https://github.com/ami-iit/bipedal-locomotion-framework/issues/316)
- Implement a matioCpp-based strain2 sensors' FT-IMU logger example device (https://github.com/ami-iit/bipedal-locomotion-framework/pull/326)
- Implement `TomlImplementation` in `ParametersHandler` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/328)
- Implement blf_calibration_delta_updater.py application (https://github.com/ami-iit/bipedal-locomotion-framework/pull/332)

### Changed
- Move all the Contacts related classes in Contacts component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/204)
- Move all the ContactDetectors related classes in Contacts component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/209)
- The DCMPlanner and TimeVaryingDCMPlanner initialize functions take as input an std::weak_ptr. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/208)
- Use `Math::StandardAccelerationOfGravitation` instead of hardcoding 9.81. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/211)
- Convert iDynTree types in FloatingBaseEstimators component to Eigen/manif types (https://github.com/ami-iit/bipedal-locomotion-framework/pull/215)
- Use std::optional instead of raw pointer in ISensorBridge. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/226)
- Use `System::LinearTask` in TSID component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/240)
- Restructure python bindings in submodules (https://github.com/ami-iit/bipedal-locomotion-framework/pull/238)
- Integrators and DynamicalSystems are now in the `ContinuousDynamicalSystem` component (https://github.com/ami-iit/bipedal-locomotion-framework/pull/242)
- Add Input template class to `System::Advanceable` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/267)
- Add support for landmarks and kinematics-free estimation in `FloatingBaseEstimators`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/254)
- If FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES is OFF, for Python bindings use installation directory provided by sysconfig Python module. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/274)
- Reduce memory allocation in `YarpSensorBridge` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/278)
- Use `TextLogging` in `VariablesHandler` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/291)
- Fix `YarpImplementation::setParameterPrivate()` when a boolean or a vector of boolean is passed (https://github.com/ami-iit/bipedal-locomotion-framework/pull/311)
- Add `foot_take_off_acceleration` and `foot_take_off_velocity` parameters in the `SwingFootPlanner` class (https://github.com/ami-iit/bipedal-locomotion-framework/issues/323)
- Change the parameters handler verbosity (https://github.com/ami-iit/bipedal-locomotion-framework/pull/330)
- Restore backward compatibility of SwingFootPlanner parameters (https://github.com/ami-iit/bipedal-locomotion-framework/pull/334)
- Bump manif version to 0.0.4  (https://github.com/ami-iit/bipedal-locomotion-framework/pull/339)

### Fixed
- Fix missing implementation of `YarpSensorBridge::getFailedSensorReads()`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/202)
- Fixed `mas-imu-test` configuration files after FW fix.
- Fixed the implementation ``YarpSensorBridge::attachAllSixAxisForceTorqueSensors()`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/231)
- Avoid the "Generating the Urdf Model from" message to appear when doing ccmake. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/243)
- Fixed the installation path of public headers related to perception libraries. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/245)
- Fixed InstallBasicPackageFiles to avoid the same problem of https://github.com/ami-iit/matio-cpp/pull/41 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/253)
- Call `positionInterface->setRefSpeeds()` only once when a position reference is set in `YarpRobotControl` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/271)
- Fix initialization of reference frame for world in `LeggedOdometry` class. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/289)
- `LeggedOdometry::Impl::updateInternalContactStates()` is now called even if the legged odometry is not initialize. This was required to have a meaningful base estimation the first time `LeggedOdometry::changeFixedFrame()` is called. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/292)
- Avoid to use the default copy-constructor and copy-assignment operator in `ContactPhaseList` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/295)
- Fix `toString()` method of `VariablesHandler` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/302)
- Fix in `YarpUtilities::getVectorFromSearchable` when a vector of boolean is passed as input (https://github.com/ami-iit/bipedal-locomotion-framework/pull/313)
- Various fixes for the yarp devices (https://github.com/ami-iit/bipedal-locomotion-framework/pull/337)

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
- Implement `ParametersHandler` library (https://github.com/ami-iit/bipedal-locomotion-controllers/pull/13)
- Implement `GenericContainer::Vector` (https://github.com/ami-iit/bipedal-locomotion-controllers/pull/29)
- Implement `Estimators` library (https://github.com/ami-iit/bipedal-locomotion-controllers/pull/23)
- Implement `Contact` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/43 and https://github.com/ami-iit/bipedal-locomotion-framework/pull/45)
- Implement the first version of the `TimeVaryingDCMPlanner` (https://github.com/ami-iit/bipedal-locomotion-framework/pull/61)
- Implement the Quintic Spline class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/83)
- Implement the `ConvexHullHelper` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/51)
- Implement the `DynamicalSystem` and `Integrator` class (https://github.com/ami-iit/bipedal-locomotion-framework/pull/46)
- Implement the `IRobotControl` interface and the YARP specialization (https://github.com/ami-iit/bipedal-locomotion-framework/pull/97, https://github.com/ami-iit/bipedal-locomotion-framework/pull/192)
- Add `SensorBridge` interface (https://github.com/ami-iit/bipedal-locomotion-framework/pull/87)
- Add the `YarpSensorBridge` Implementation (https://github.com/ami-iit/bipedal-locomotion-framework/pull/106)
- Added `CommonConversions`, `ManifConversions`, and `matioCppConversions` libraries to handle type conversions. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/138 and https://github.com/ami-iit/bipedal-locomotion-framework/pull/143)
- Implement the `JointPositionTracking` application. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/136)
- Initial implementation of Python bindings using pybind11 (https://github.com/ami-iit/bipedal-locomotion-framework/pull/134)
- Implement `FloatingBaseEstimatorDevice` YARP device for wrapping floating base estimation algorithms. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/130)
- Implement Continuous algebraic Riccati equation function (https://github.com/ami-iit/bipedal-locomotion-framework/pull/157)
- Implement YARP based `ROSPublisher` in the `YarpUtilities` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/156)
- Implement example YARP device `ROSPublisherTestDevice` for understanding the usage of `ROSPublisher`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/160)
- Implement `TSID` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/167, https://github.com/ami-iit/bipedal-locomotion-framework/pull/170, https://github.com/ami-iit/bipedal-locomotion-framework/pull/178)
- Implement the `JointTrajectoryPlayer` application. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/169)29ed234a1c
- Implement `ContactDetectors` library. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/142)
- Added `mas-imu-test` application to check the output of MAS IMUs (https://github.com/ami-iit/bipedal-locomotion-framework/pull/62)
- Implement motor currents reading in `YarpSensorBridge`. (https://github.com/ami-iit/bipedal-locomotion-framework/pull/187)

[unreleased]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.18.0...master
[0.18.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.17.0...v0.18.0
[0.17.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.16.1...v0.17.0
[0.16.1]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.16.0...v0.16.1
[0.16.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.15.0...v0.16.0
[0.15.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.14.1...v0.15.0
[0.14.1]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.14.0...v0.14.1
[0.14.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.13.0...v0.14.0
[0.13.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.12.0...v0.13.0
[0.12.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.11.1...v0.12.0
[0.11.1]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.11.0...v0.11.1
[0.11.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.10.0...v0.11.0
[0.10.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/ami-iit/bipedal-locomotion-framework/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/ami-iit/bipedal-locomotion-framework/releases/tag/v0.1.0
