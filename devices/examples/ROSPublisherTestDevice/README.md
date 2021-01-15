### ROS Publisher Test Device

- Run the servers 

  ```bash
  roscore
  yarpserver --ros
  yarpdev --device transformServer --name tfServer --ROS::enable_ros_publisher true --ROS::enable_ros_subscriber true
  ```

  (This step can also be done using [ros-publisher-test-runner.xml](./ros-publisher-test-runner.xml) through the yarpmanager).

- Run the device using

  ``` bash
  YARP_ROBOT_NAME=iCubGazeboV2_5 yarprobotinterface --config launch-ros-publisher-test.xml
  ```

- Check the outputs of the test device by running,

  ``` bash
  rostopic echo /joint_states    # to check the published joint states - name: [hello] position: [20] 
  rostopic echo /wrench/right    # to check the published wrench - frame: right force: x:0.0 y:1.0 z:2.0 torque: x:0.0 y:0.0 z:0.0
  rostopic echo /tf # to check the published pose between /world and /dummy as identity (zero position and unit quaternion)
  ```

  

