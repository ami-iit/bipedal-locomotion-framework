# bipedal-locomotion-controllers <a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard" /></a>  </a><a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>

The **bipedal-locomotion-controllers** project is a _suite_ of libraries for achieving bipedal locomotion on humanoid robots.


# Overview
- [:orange_book: Exported components](#orange_book-exported-components)
- [:page_facing_up: Dependencies](#page_facing_up-dependencies)
- [:hammer: Build the suite](#hammer-build-the-suite)
- [:running: How to use the libraries](#running-how-to-use-the-libraries)
- [:books: Doxigen documentation](#books-doxigen-documentation)

# :orange_book: Exported components
- `BipedalLocomotionControllers`: It is an _interface_ library that gathers all
  the exported components.
- [**YarpUtilities**](./src/YarpUtilities): Utilities library for retrieving
  data and from YARP structures
- [**OptimalControlUtilities**](./src/OptimalControlUtilities): Utilities library for handling cost and
  constraints in a quadratic optimization problem (QP)


# :page_facing_up: Dependencies
The **bipedal-locomotion-controllers** project is versatile and it can be used
to compile only some components. Each component has its own dependencies that
can be found in [`BipedalLocomotionControllersFindDependencies.cmake`](./cmake/BipedalLocomotionControllersFindDependencies.cmake)
file. Please note that the indicated version is the minimum required version.

- `YarpUtilities` requires:
    - For using it:
      - [`iDynTree`](https://github.com/robotology/idyntree) (version 0.11.105)
      - [`YARP`](https://github.com/robotology/YARP)
    - For testing:
      - [`Catch2`](https://github.com/catchorg/Catch2)
- `OptimalControlUtilities` requires:
    - [`iDynTree`](https://github.com/robotology/idyntree) (version 0.11.105)

# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/dic-iit/bipedal-locomotion-controllers.git
cd bipedal-locomotion-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
```sh
export BipedalLocomotionControllers_INSTALL_DIR=/path/where/you/installed/
export PATH=$PATH:$BipedalLocomotionControllers_INSTALL_DIR/lib
```
# :running: How to use the libraries
bipedal-locomotion-controllers provides native CMake support which allows the library to be easily used in CMake projects.

bipedal-locomotion-controllers exports the `CMake` targets presented in
[Exported components](#orange_book-exported-components) section. The targets can
be imported using the `find_package` CMake command and used by calling
`target_link_libraries`.
For instance, the `OptimalControlUtilities` component can be used as follows:
```cmake
cmake_minimum_required(VERSION 3.0)
project(myproject)
find_package(BipedalLocomotionControllers REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example BipedalLocomotionControllers::OptimalControlUtilities)
```

# :books: Doxigen documentation
[Here](https://dic-iit.github.io/bipedal-locomotion-controllers) you can find the documentation.
