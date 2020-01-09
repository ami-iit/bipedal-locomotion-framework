# bipedal-locomotion-controllers <a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard" /></a>  </a><a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>

The **bipedal-locomotion-controllers** project is a _suite_ of libraries for achieving bipedal locomotion on humanoid robots.


# Overview
- [:orange_book: Exported components](#orange_book-exported-components)
- [:page_facing_up: Dependencies](#page_facing_up-dependencies)
- [:hammer: Build the suite](#hammer-build-the-suite)
- [:running: How to use the libraries](#running-how-to-use-the-libraries)

# :orange_book: Exported components

# :page_facing_up: Dependencies


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

bipedal-locomotion-controllers exports the `CMake` targets presented in [Exported components](#orange_book-exported-components) section. The targets can be imported using the `find_package` CMake command and used by calling `target_link_libraries`.
