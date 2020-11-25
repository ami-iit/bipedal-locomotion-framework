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
in a terminal. The test should run automatically.

#### Insert here some real output

The test has also an RPC interface. It is possible to launch it by running
```
yarp rpc /masImuTest/rpc
```
in a terminal. Then the following commands are available:
- ``quit`` Quits the module.
- ``startTest`` Manually start the test.
- ``stopTest`` Manually stop the test.
- ``printResults`` If the test is stopped, print the results.

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

The following part of the configuration file contains two part which are equivalent, one for the left leg, having the tag ``[LEFT_LEG]`` and one for the right leg. We will detail here only the part for the left leg since the other is equivalent.
- ``remote    left_leg/inertials`` The remote port from where the output of the IMU is streamed.
- ``imu_frame    l_foot_ft_acc_3b13`` The name of the frame in the URDF corresponding to the IMU to check.
- ``imu_sensor_name    l_foot_ft_eul_3b13`` The name of the IMU sensor to check.
- ``gyro_sensor_name    l_foot_ft_gyro_3b13`` The name of the gyro sensor attached to the IMU sensor.
- ``acc_sensor_name    l_foot_ft_acc_3b13`` The name of the accelerometer attached to the IMU sensor.
- ``remote_control_boards    ("left_leg")`` The comma-separated list of control boards including all the joints connecting the sensor under testing to the base link.
- ``rpy_shuffling ("roll", "pitch", "yaw")`` It allows shuffling the rpy order. Use only the "roll", "pitch", "yaw" keywords. It is possible to prepend "-", e.g. "-pitch" to change the sign.
