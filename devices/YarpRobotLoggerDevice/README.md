# YARPRobotLoggerDevice

The **YARPRobotLoggerDevice** is a YARP device based on `YarpSensorBridge` and `matioCpp` to record dataset from a YARP-based robot.



## :running: How to use the device

- Launch `yarprobotinterface` on the robot.

  Depending on your logger configuration file and on the data you want to collect you should run the main `yarprobotinterface` with a different option. In order to use the [defaul logger configuration files](./app/robots), use the following commands:
  - **iCubGenova04**

    launch the `yarprobotinterface` with the `root_imu` device on the robot head
    ```
    yarprobotinterface --config icub_wbd_inertials.xml
    ```
  - **iCubGazeboV3**

    open the robot model in gazebo and launch `whole-body-dynamics`
    ```
    YARP_ROBOT_NAME=iCubGazeboV3 yarprobotinterface --config launch-wholebodydynamics.xml
    ```
    in case you started gazebo with the real-time clock option (`gazebo -slibgazebo_yarp_clock.so`) add `YARP_CLOCK=/clock` to the previous command.
    ```
    YARP_CLOCK=/clock YARP_ROBOT_NAME=iCubGazeboV3 yarprobotinterface --config launch-wholebodydynamics.xml
    ```

- Launch the logger device.

  In order to use the [defaul logger configuration files](./app/robots), set the `YARP_ROBOT_NAME` environment variable and launch the `yarprobotinterface` as follow
  ```
  yarprobotinterface --config launch-yarp-robot-logger.xml
  ```

- Press Ctrl+c to close the device, and the dataset is stored as the device is closed.

## 💾 Data

 Depending on the configuration file, each dataset can contain:
 - `time` vector with the YARP clock time stamps.
 - `joint_state` that containins
   - `joints`: name of the joints
   - `joint_positions`
   - `joint_velocities`
   - `joint_torques`
- `Motor_state` that contains
  - `motor_currents`
  - `motor_velocities`
  - `motor_positions`
- `PID`
- `Motor_PWM`
- `Accelerometer` struct containing data for each selected source.
- `CartesianWrench` struct containing data for each selected source.
- `FT` struct containing data for each selected source.
- `Gyros` struct containing data for each selected source.
- `Orientation` struct containing data for each selected source as roll-pitch-yaw euler angles.

## 🏷️ Migrate mat files
### v0.6.0 -> v0.7.0

To migrate an existing `mat` file saved with the device version `v0.6.0` to `v0.7.0` you can run the matlab script `scripts/migrate_0_6_0_to_0_7_0.m` as
```matlab
% name of the robot
robot_name = 'iCubGenova09';

% path of the folder containing the mat files that will migrate
files_path = './dataset';
migrate_0_6_0_to_0_7_0(files_path, robot_name);
```
You will find the new datasets in `<files_path>/v070`
