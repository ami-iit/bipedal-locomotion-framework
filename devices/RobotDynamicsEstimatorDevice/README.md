# RobotDynamicsEstimatorDevice

The **RobotDynamicsEstimatorDevice** is a YARP device based on `RobotDynamicsEstimator` to estimate joint torques from a YARP-based robot.


## :running: How to use the device

To run the device on a robot (whether in simulation or on a real robot), make sure to define the configuration files for the robot in the `app/robots` folder.

- **ergoCubGazeboV1**

  - open the robot model in Gazebo
  - launch `RobotDynamicsEstimatorDevice`
    ```
    YARP_ROBOT_NAME=ergoCubGazeboV1 yarprobotinterface --config launch-robot-dynamics-estimator.xml
    ```
    in case you started gazebo with the real-time clock option (`gazebo -slibgazebo_yarp_clock.so`) add `YARP_CLOCK=/clock` to the previous command.
    ```
    YARP_CLOCK=/clock YARP_ROBOT_NAME=ergoCubGazeboV1 yarprobotinterface --config launch-robot-dynamics-estimator.xml
    ```
- **ergoCubSN000**

  - Launch `yarprobotinterface` on the robot.
  - launch `RobotDynamicsEstimatorDevice`
    ```
    YARP_ROBOT_NAME=ergoCubSN000 yarprobotinterface --config launch-robot-dynamics-estimator.xml
    ```
