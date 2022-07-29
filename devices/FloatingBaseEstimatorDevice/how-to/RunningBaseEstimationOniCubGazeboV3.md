

### Prerequisites

1. Create  a file `gazebo_icub_base_state.ini` in `icub-models/iCub/conf_icub3` which contains the following lines,

      ``` ini
      [WRAPPER]
      name /icubSim/floating_base/state:o
      period 1
      device analogServer
      
      [DRIVER]
      device gazebo_basestate
      baseLink chest
      ```

      to get the ground truth pose of the chest link from Gazebo streamed on a port.

      One may also similarly add the plugin for `x_foot_rear` to then compute the pose of `x_sole` frame.

      ``` ini
      [WRAPPER]
      name /icubSim/r_foot_rear/state:o
      period 1
      device analogServer
      
      [DRIVER]
      device gazebo_basestate
      baseLink r_foot_rear
      ```

2. Add these as plugins to the  `model.urdf` in `icub-models/iCub/robots/iCubGazeboV3` by adding the following lines in the file within the `<robot> .... </robot> `tags,

      ``` xml
      <gazebo>
      <plugin name="base_state" filename="libgazebo_yarp_basestate.so">
            <yarpConfigurationFile>model://iCub/conf_icub3/gazebo_icub_base_state.ini</yarpConfigurationFile>
      </plugin>
      </gazebo>
      ```

Optionally, also add,

```xml
<plugin name="l_foot_rear_state" filename="libgazebo_yarp_basestate.so">
<yarpConfigurationFile>model://iCub/conf_icub3/gazebo_icub_l_foot_rear_state.ini</yarpConfigurationFile>
</plugin>
       <plugin name="r_foot_rear_state" filename="libgazebo_yarp_basestate.so">
<yarpConfigurationFile>model://iCub/conf_icub3/gazebo_icub_r_foot_rear_state.ini</yarpConfigurationFile>
</plugin>
```

3. Install the `icub-models` by running a cmake and make install to update the files at the install location. (If the changes you made are not reflected while launching Gazebo, it is mostly due to not running a cmake.)

4. Create an `icub3.world` (if not available in `bipedal-locomotion-framework/devices/FloatingBaseEstimatorDevice/gazebo/worlds`) by pasting the lines,

``` xml
<?xml version="1.0" ?>

      <sdf version="1.5">
      	<world name="default">
      		<!-- A global light source -->
      		<include>
      			<uri>model://sun</uri>
      		</include>
      
      		<!-- A ground plane -->
      		<include>
      			<uri>model://ground_plane</uri>
      		</include>

      		<!-- iCub -->
          		<include>
      		        <uri>model://iCubGazeboV3</uri>
      		</include>
      	</world>
      </sdf>
```

5. Add the world to the Gazebo resource path by adding the following line to the `.bashrc` file,

``` bash
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<path-to-bipedal-locomotion-framework/devices/FloatingBaseEstimatorDevice/gazebo/worlds>
```

6. Run `yarpserver` and launch Gazebo using,

   ``` bash
   gazebo --slibgazebo_yarp_clock.so icub3.world
   ```

7. Ensure that the base state is being streamed by checking,
``` bash
yarp read ... /icubSim/floating_base/state:o
```
The first 6 values in the streamed vector represent xyz position in meters and rpy orientation in radians of the chest link in Gazebo world (see https://github.com/robotology/gazebo-yarp-plugins/blob/d1995eac52adf6a273185da3eab2f279718725b9/plugins/basestate/include/yarp/dev/BaseStateDriver.h#L49-L51 for unit conventions). This information will be useful in setting the initial states for the EKF.  Alternately, one can write a script/application to read the pose from the transform server and use this to reset the base state of the EKF through an RPC call which can be convenient while loading the model in different initial positions within the simulator.

8. When the iCubGazeboV3 model is loaded in the zero position of the Gazebo world, the chest link has the pose `pos = [-0.0111; 0.0; 0.8299] rpy = [0.0; 0.001; 0.0007] quat_wxyz = [1.0000; 0.0; 0.0005; 0.0003]` , thus the after applying the chest-to-imu transformation, the pose of the chest imu in the world is `pos = [0.0784; 0.0176; 0.9291] rpy = [-1.5718; 0.0; -1.5718] quat_wxyz = [0.4999; -0.5004; 0.5001; -0.4996]`.  When passing the reference frame as `chest` for world for `LeggedOdom`, the chest pose must be inverted and set in the configuration to set the pose of the world with respect to the reference frame that is the chest.

SImilarly, `l_foot_rear` pose in world is `pos = [-0.0108; 0.0647;0.0]. rpy = [0.0, 0.0, 0.0]` which leads to `l_sole`pose as `pos = [0.04879; 0.0647;0.0]. rpy = [0.0, 0.0, 0.0]`. Subsequently, `r_sole` pose is `pos = [0.04879; -0.0647;0.0]. rpy = [0.0, 0.0, 0.0]`.

These data will be used to fill the `InitialStates` subgroup in the `BaseEstimator` parameter group for the EKF configuration.

10. Prepare the Vector Collections Wrapper Device for collecting the ground truth base state from Gazebo and estimated base state from the EKF within a single port in order to be logged by the `YarpRobotLoggerDevice`.  `icub3-vc-wrapper-sim.xml` reads as,
    ``` xml
    <!-- Copyright (C) 2019-2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    This software may be modified and distributed under the terms of the
    BSD-3-Clause license. -->
    
    <?xml version="1.0" encoding="UTF-8" ?>
    <device  xmlns:xi="http://www.w3.org/2001/XInclude" name="icub3-sim-vectors-collection-wrapper" type="VectorsCollectionWrapper">
      <param name="sampling_period_in_s">0.01</param>
      <param name="port_prefix">/icubSim/baseState/vcWrapper</param>
      <param name="remote_port_names">("/icubSim/floating_base/state:o", "/base-estimator/floating_base/state:o")</param>
      <param name="remote_var_names">("sim_base_state", "est_base_state")</param>
      <param name="output_port_name">"/vcWrapper/floating_base_state"</param>
    </device>
    
    ```

    while, `launch-icub3-vc-wrapper-sim.xml` reads as,

    ``` xml
    <!-- Copyright (C) 2019-2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    This software may be modified and distributed under the terms of the
    BSD-3-Clause license. -->
    
    <?xml version="1.0" encoding="UTF-8" ?>
    <!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
    
    <robot name="iCubGazeboV3" portprefix="icubSim" build="1" xmlns:xi="http://www.w3.org/2001/XInclude">
        <devices>
            <xi:include href="./interface/icub3-vc-wrapper-sim.xml" />
        </devices>
    </robot>
    ```

11. Prepare the `YarpRobotLoggerDevice` to log the relevant output from Gazebo and Base EKF.

12. Added left_leg_ft_sensor to the URDF model in order to run WBD.

    ``` xml
      <gazebo reference="l_leg_ft_sensor">
        <sensor name="l_leg_ft_sensor" type="force_torque">
          <always_on>1</always_on>
          <update_rate>100</update_rate>
          <force_torque>
            <frame>sensor</frame>
            <measure_direction>child_to_parent</measure_direction>
          </force_torque>
          <pose>0.0 0.0 0.0 3.141592653589793 0.0 -2.6179934742070086</pose>
          <plugin name="left_leg_ft_plugin" filename="libgazebo_yarp_forcetorque.so">
      <yarpConfigurationFile>model://iCub/conf_icub3/FT/gazebo_icub_left_leg_ft.ini</yarpConfigurationFile>
    </plugin>
        </sensor>
      </gazebo>
      <sensor name="l_leg_ft_sensor" type="force_torque">
        <parent joint="l_leg_ft_sensor"/>
        <force_torque>
          <frame>sensor</frame>
          <measure_direction>child_to_parent</measure_direction>
        </force_torque>
        <origin rpy="3.141592653589793 0.0 -2.6179934742070086" xyz="0.0 0.0 0.0"/>
      </sensor>
    
    ```

### Launching

1. `yarpserver`
2. `gazebo --slibgazebo_yarp_clock.so icub3.world`
3. `YARP_ROBOT_NAME=iCubGazeboV3 yarprobotinterface --config launch-wholebodydynamics.xml`
4. `WalkingModule`
5. `yarprobotinterface --config launch-base-estimator.xml`
6. `yarprobotinterface --config launch-icub3-vc-wrapper-sim.xml`
7. Launch logger (to be tested with the recent changes in the `YarpRobotLoggerDevice`)

### Troubleshooting

1. If `YarpRobotLoggerDevice` fails to compile with the following error,
   ``` bash
   bipedal-locomotion-framework/devices/YarpRobotLoggerDevice/include/BipedalLocomotion/YarpRobotLoggerDevice.h:16:10: fatal error: opencv2/opencv.hpp: No such file or directory
      16 | #include <opencv2/opencv.hpp>
         |          ^~~~~~~~~~~~~~~~~~~~
   compilation terminated.
   make[2]: *** [devices/YarpRobotLoggerDevice/CMakeFiles/YarpRobotLoggerDevice.dir/build.make:76: devices/YarpRobotLoggerDevice/CMakeFiles/YarpRobotLoggerDevice.dir/src/YarpRobotLoggerDevice.cpp.o] Error 1
   make[1]: *** [CMakeFiles/Makefile2:1797: devices/YarpRobotLoggerDevice/CMakeFiles/YarpRobotLoggerDevice.dir/all] Error 2
   make[1]: *** Waiting for unfinished jobs....
   ```

   do `sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2`

or wherever `opencv4` is installed, for example if installed with conda in `robsub` environment,

``` bash
sudo ln -s /home/user/mambaforge/envs/robsub/include/opencv4/opencv2 /home/user/mambaforge/envs/robsub/include/opencv2
```

2. If there is position error in x-direction, then the Schmitt Trigger parameters need to be tuned for proper contact detection.
3. If there is orientation errors, then there is a problem with the measurements coming form the Gazebo IMU. Specifically, they are not rotated into the IMU frame.