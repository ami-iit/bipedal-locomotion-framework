# YARPRobotLoggerDevice

The **YARPRobotLoggerDevice** is a YARP device based on `YarpSensorBridge` and `matioCpp` to record dataset from a YARP-based robot.



## :running: How to use the device

- Launch yarprobotinterface on the robot 

- Launch the logger device after properly setting the `YARP_ROBOT_NAME` environment variable
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

