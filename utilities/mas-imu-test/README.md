# MAS IMU Test

The ``mas-imu-test`` was designed with the intent of testing whether the orientation provided by the IMU sensors is coherent with the encoder measurements.
The test requires the operator to move manually the limbs where the sensors are located. In this way, the test acquires data to detect if the IMU measurements and the encoder measurements coincide.

## Installation
The ``mas-imu-test`` is part of the ``BipedalLocomotionFramework``. It is compiled if the ``CMake`` option ``FRAMEWORK_COMPILE_MasImuTest`` is set to ``ON``. To install, follow the instructions for ``BipedalLocomotionFramework``. In order for the executable to be easily launched, add the folder
```
/path/to/install/bin
```
to the path (substitute ``/path/to/install`` with the installation directory of bipedal locomotion framework).
The test also requires some configuration files. In order to find the correct configuration file, set the environmental variable ``YARP_ROBOT_NAME`` with the name of the robot on which you want to perform the test.

:warning: In the ``BipedalLocomotionFramework`` repo only the files for ``iCubGenova04`` and ``iCubGazeboV2_5`` are present.

In order for the configuration files to be found, it is necessary to add the following directory
```
/path/to/install/share/BipedalLocomotionFramework
```
to the ``YARP_DATA_DIRS`` environmental variable.

## Running the test
Simply run
```
blf-mas-imu-test
```
in a terminal. The test should run automatically. Then, move the limbs (in the way you prefer) where each IMU you want to test is located. 
For each IMU to test, the module prints a status message like
```
[INFO][MasImuTest::MasImuData::addSample](Left IMU Test)  Sample  3 / 500 
```
every time a joint in the kinematic chain from the root link to the IMU is moved.
When all the tests are completed, a summary similar to the following is printed.
```
[INFO][MasImuTest::MasImuData::printResults](Left IMU Test)  Inertial calibration matrix:
 --------------------------------------
 0.99305 -0.0163568 0.116548
0.00289557 -0.986599 -0.163135
0.117654 0.162339 -0.979696
 RPY [deg]: (170.591379, -6.756744, 0.167065)
 --------------------------------------
 Results ( 500  samples) :
 --------------------------------------
 --------------Mean Rotation-----------
 0.961574 0.274212 0.0134994
-0.258299 0.88692 0.382954
0.0930379 -0.371726 0.923669
 RPY [deg]: (-21.922084, -5.338397, -15.035917)
 ----------------Min Error-------------
 Index:  358 
 0.999966 -0.000548794 0.0082332
0.00045783 0.999939 0.0110462
-0.00823876 -0.0110421 0.999905
 RPY [deg]: (-0.632700, 0.472051, 0.026233)
 ----------------Max Error-------------
 Index:  153 
 0.31468 0.103543 0.943533
-0.00670651 -0.993765 0.111292
0.949174 -0.0413493 -0.312024
 RPY [deg]: (-172.451167, -71.654180, -1.220911)
 --------------------------------------
 ```

When the test closes normally, it saves a ``.mat`` file with all the data and the settings saved in.

The test has also an RPC interface. It is possible to launch it by running
```
yarp rpc /masImuTest/rpc
```
in a terminal. Then the following commands are available:
- ``quit`` Quits the module.
- ``startTest`` Manually start the test.
- ``stopTest`` Manually stop the test.
- ``printResults`` If the test is stopped, print the results.

## Plotting
In the folder ``scripts``, a Matlab script is provided. Load in the workspace the saved ``.mat`` file and simply run the script ``plotResults`` from Matlab.
You may need to edit these lines
```matlab
%% Settings

robotName='iCubGenova04'; %% Name of the robot

meshFilePrefix = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share']; %% Path to the model meshes

modelPath = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share/iCub/robots/' robotName '/'];  %% Path to the robot model

fileName='model.urdf'; %% Name of the urdf file
```
according to your environment. If you installed ``BipedalLocomotionFramework`` via the ``robotology-superbuild``, you may need to change only the robot name.

The script should start playing the data on a 3D version of the robot. You should be able to see two frames. One with long and thin axis is the expected frame orientation from forward kinematics. 
The other has short and thick axis. This is the estimated one via the IMU. These two should match.
In addition, you should be able to see a magenta line indicating the accelerometer measurement (scaled).

It also prints the expected RPY values vs the measured ones.

### Example of not aligned frames

(The magenta vector is disabled for this image.)

![ezgif-2-38372e5d4323](https://user-images.githubusercontent.com/18591940/100088488-7a5b5780-2e50-11eb-8b85-603f806f8105.gif)

### Example of aligned frames

![left](https://user-images.githubusercontent.com/18591940/100474051-0cc55a80-30e0-11eb-9e45-1b4f95820bf3.gif)

## Configuration file explanation
The configuration file presents the following data:
- ``name    masImuTest`` The name of the test, used to define the name of the opened ports.
- ``robot    icub`` The prefix of the ports opened by the robot to which the test has to connect.
- ``model    model.urdf`` The name of the ``urdf`` model.
- ``period    0.01`` The period at which the module runs.
- ``base_link    root_link`` The link of the robot to be considered fixed.
- ``base_rotation    ((-1.0 0.0 0.0),(0.0 -1.0 0.0),(0.0 0.0 1.0))``. The rotation of the base link with respect to an inertial frame having the z-axis pointing against gravity and the x axis defining the forward direction. The rotation is defined by rows
- ``filter_yaw    true`` Boolean value used to trigger the filtering of the yaw measurement coming from the IMU. This measurement is substituted by the one obtained through forward kinematics.
- ``min_joint_variation_deg    2.0`` Minimum joint variation after which a new sample is taken into consideration.
- ``max_samples    500`` The max number of samples considered in the test.
- ``mas_timeout    0.02`` Timeout for reading the MAS IMU sensor. A warning is thrown if this timeout is not respected.
- ``auto_start    true`` The test start automatically without having to use the RPC interface.
- ``file_name     masImuTestOutput.mat`` The name of the mat file where data is logged.
- ``tests     ("LEFT_LEG", "RIGHT_LEG")`` The set of tests to be performed. The name should correspond to a group with the structure described below.


The following part of the configuration file contains two part which are equivalent, one for the left leg, having the tag ``[LEFT_LEG]`` and one for the right leg. We will detail here only the part for the left leg since the other is equivalent.
- ``pretty_name    "Left IMU Test"`` A pretty name for the test
- ``log_prefix    left`` The name used for the test in the logged data (it should start with a letter, and contain only alphanumeric charachters or "_").
- ``remote    left_leg/inertials`` The remote port from where the output of the IMU is streamed.
- ``imu_frame    l_foot_ft_acc_3b13`` The name of the frame in the URDF corresponding to the IMU to check.
- ``imu_sensor_name    l_foot_ft_eul_3b13`` The name of the IMU sensor to check.
- ``gyro_sensor_name    l_foot_ft_gyro_3b13`` The name of the gyro sensor attached to the IMU sensor.
- ``acc_sensor_name    l_foot_ft_acc_3b13`` The name of the accelerometer attached to the IMU sensor.
- ``remote_control_boards    ("left_leg")`` The comma-separated list of control boards including all the joints connecting the sensor under testing to the base link.
- ``rpy_shuffling ("roll", "pitch", "yaw")`` It allows shuffling the rpy order. Use only the "roll", "pitch", "yaw" keywords. It is possible to prepend "-", e.g. "-pitch" to change the sign.
