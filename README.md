<img src="https://user-images.githubusercontent.com/16744101/108218725-3cab5980-7135-11eb-8b5a-8bcc9226fd5a.png" align="right" />

<h1 align="center">bipedal-locomotion-framework</h1>

<p align="center">
   <a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard" class="center"/></a>
   <a href="https://github.com/dic-iit/bipedal-locomotion-framework/blob/master/LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" class="center"/></a>
  <a href="https://dic-iit.github.io/bipedal-locomotion-framework/doxygen/doc/html/index.html"><img src="https://github.com/dic-iit/bipedal-locomotion-framework/workflows/GitHub%20Pages/badge.svg" alt="doxygen"/></a>
  <a href="https://github.com/dic-iit/bipedal-locomotion-framework/actions?query=workflow%3A%22C%2B%2B+CI+Workflow%22"><img src="https://github.com/dic-iit/bipedal-locomotion-framework/workflows/C++%20CI%20Workflow/badge.svg" alt="CI"/></a>
</p>


---

<p align="center">
  <b>:warning: REPOSITORY UNDER DEVELOPMENT :warning:</b>
  <br>The libraries implemented in this repository are still experimental and we cannot guarantee stable API
</p>




---



The **bipedal-locomotion-framework** project is a _suite_ of libraries for achieving bipedal locomotion on humanoid robots.
# Table of content

- [:page_facing_up: Mandatory dependencies](#page_facing_up-mandatory-dependencies)
- [:hammer: Build the suite](#hammer-build-the-suite)
- [:orange_book: Exported components](#orange_book-exported-components)
- [:computer: Some utitilites](#computer-some-utilities)
- [:snake: Python](#snake-python)
- [:running: How to use the libraries](#running-how-to-use-the-libraries)
- [:gear: Contributing​](#gear-contributing)

# :page_facing_up: Mandatory dependencies
The **bipedal-locomotion-framework** project is versatile and can be used to compile only some components.

The minimum required dependencies are `Eigen3`, `iDynTree` and `spdlog`. If you want to build the tests please remember to install `Catch2`. If you are interested in the python bindings generation please install `python3` and `pybind11` in your system.

# :orange_book: Exported components

The **bipedal-locomotion-framework** project consists of several components. The components are stored in the [`src`](./src) folder and their compilation depends on the installed dependencies.

|                  Component                   |                         Description                          |                   Additional Dependencies                    |
| :------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
|          [AutoDiff](./src/Autodiff)          |                Bridge between CppAD and Eigen                |   [`CppAD`](https://coin-or.github.io/CppAD/doc/cppad.htm)   |
|     [ContactModels](./src/ContactModels)     | Models to describe the contact between robot and enviroment  |                              -                               |
|          [Contacts](./src/Contacts)          |              Syntactic description of a contact              |         [`manif`](https://github.com/artivis/manif)          |
|    [CommonConversions](./src/Conversions)    |      Common conversion utilities used in the framework       |                              -                               |
|    [ManifConversions](./src/Conversions)     | `manif` library related conversion utilities used in the framework |         [`manif`](https://github.com/artivis/manif)          |
|        [Estimators](./src/Estimators)        |                 Library containing observers                 |                              -                               |
|  [FloatingBaseEstimator](./src/Estimators)   |         Library containing floating base estimators          |         [`manif`](https://github.com/artivis/manif)          |
|  [GenericContainer](./src/GenericContainer)  |      Data structure similar to ``span`` but resizable.       |                              -                               |
|               [IK](./src/IK)                 |                      Inverse kinematics                      | [`manif`](https://github.com/artivis/manif) [`osqp-eigen`](https://github.com/robotology/osqp-eigen) |
|              [Math](./src/Math)              |          Library containing mathematical algorithms          |                              -                               |
| [ParametersHandler](./src/ParametersHandler) |  Library for retrieving parameters from configuration files  | [`YARP`](https://www.yarp.it/git-master/) (only if you want the `YARP` implementation) |
|          [Planners](./src/Planners)          |       Library containing planner useful for locomotion       | [`manif`](https://github.com/artivis/manif) [`CasADi`](https://web.casadi.org/) [`qhull`](http://www.qhull.org/) |
|    [RobotInterface](./src/RobotInterface)    | Generic interface classes to adapt to various IO data formats | [`YARP`](https://www.yarp.it/git-master/) (only if you want the `YARP` implementation) |
|            [System](./src/System)            |   Description of discrete and continuous dynamical systems   |                              -                               |
|              [TSID](./src/TSID)              |                 Task space inverse dynamics                  | [`manif`](https://github.com/artivis/manif) [`lie-group-controllers`](https://github.com/dic-iit/lie-group-controllers) |
|     [YarpUtilities](./src/YarpUtilities)     | Utilities library for retrieving data and from YARP structures |          [`YARP`](https://www.yarp.it/git-master/)           |
| [PerceptionInterface](./src/RobotInterface)  | Generic interface classes to adapt to perception data formats like images and point clouds | [`OpenCV`](https://github.com/opencv/opencv) [`PCL`](https://github.com/PointCloudLibrary/pcl) |
|    [PerceptionCapture](./src/Perception)     |   Library containing driver classes for perception devices   | [`OpenCV`](https://github.com/opencv/opencv) [`PCL`](https://github.com/PointCloudLibrary/pcl) [`realsense2`](https://github.com/IntelRealSense/librealsense) |


# :hammer: Build the suite
The **bipedal-locomotion-framework**  can be built on Windows, macOS, and Linux. The easiest way to compile the library is to use the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild). If you enable the profiles  `ROBOTOLOGY_ENABLE_DYNAMICS` and `ROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS` in the `robotology-superbuild` you will automatically clone and build **bipedal-locomotion-framework** and all the dependencies.

If you do not want to use the `robotology-superbuild` you can manually compile the code in the repository running the following command in the terminal

```sh
git clone https://github.com/dic-iit/bipedal-locomotion-framework.git
cd bipedal-locomotion-framework
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<path/where/you/want/to/install> \
      -DCMAKE_BUILD_TYPE=Release \
cmake --build . --config Release --target install
```

**Remark:** If you manually compile the framework remember to install the dependencies required by the component you are interested in.

# :computer: Some utilities

The **bipedal-locomotion-framework** ships also some utilities that can help you in the everyday tests on a real robot. You can find them in the [`utilities` folder](./utilities). Each utility contains a well-documented` README` where you can find further details.

# :snake: Python

**bipedal-locomotion-framework** provides also python bindings. Only a small set of the components implemented in the library have the corresponding python bindings.

If you want to compile the bindings please install `python3` and `pybind11` in your system then you can run the following `CMake` command in your `build` folder

```sh
cmake -DCMAKE_INSTALL_PREFIX=<path/where/you/want/to/install> \
      -DCMAKE_BUILD_TYPE=Release \
      -DFRAMEWORK_COMPILE_PYTHON_BINDINGS:BOOL=ON \
      -DPython3_ROOT_DIR=$(python3 -c "import sys; print(sys.prefix)") \
      -DFRAMEWORK_USE_Python3:BOOL=ON \
      -DFRAMEWORK_USE_pybind11:BOOL=ON .
cmake --build . --config Release --target install
```

**Disclaimer:** The python bindings are currently supported on Linux.

# :running: How to use the libraries
The **bipedal-locomotion-framework** provides native `CMake` support which allows the library to be easily used in `CMake` projects.

**bipedal-locomotion-framework** exports the `CMake` targets presented in [Exported components](#orange_book-exported-components) section. The targets can be imported using the `find_package`  command and used by calling `target_link_libraries`.

For instance, `Math` component can be used as follows:

```cmake
cmake_minimum_required(VERSION 3.0)
project(myproject)
find_package(BipedalLocomotionFramework COMPONENTS Math REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE BipedalLocomotion::Math)
```

# :gear: Contributing

**bipedal-locomotion-framework** is an open-source project, and is thus built with your contributions. We strongly encourage you to open an issue with your feature request. Once the issue has been opened, you can also proceed with a pull-request :rocket:
