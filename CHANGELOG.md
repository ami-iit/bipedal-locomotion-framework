# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
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
- Renamed from ``bipedal-locomotion-controllers`` to ``bipedal-locomotion-framework`` (https://github.com/dic-iit/bipedal-locomotion-framework/pull/40).
- Implement `Contact` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/43 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/45)
- Added `CommonConversions`, `ManifConversions`, and `matioCppConversions` libraries to handle type conversions. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/138 and https://github.com/dic-iit/bipedal-locomotion-framework/pull/143)
- Implement the `JointPositionTracking` application. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/136)
- Initial implementation of Python bindings using pybind11 (https://github.com/dic-iit/bipedal-locomotion-framework/pull/134)
- Implement `FloatingBaseEstimatorDevice` YARP device for wrapping floating base estimation algorithms. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/130)
- Implement Continuous algebraic Riccati equation function (https://github.com/dic-iit/bipedal-locomotion-framework/pull/157)
- Implement YARP based `ROSPublisher` in the `YarpUtilities` library. (https://github.com/dic-iit/bipedal-locomotion-framework/pull/156)

[Unreleased]: https://github.com/dic-iit/bipedal-locomotion-framework/
