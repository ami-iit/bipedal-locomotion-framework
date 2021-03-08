### iCubGenova04 strain2 FT-IMU logger

A logger device based on `YarpSensorBridge` and `matioCpp` to record dataset from strain2 sensor boards mounted on the iCubGenova04 robot. 



### Launching

- Launch yarprobotinterface on the robot with `icub_wbd_inertials.xml`

  ```
  yarprobotinterface --config icub_wbd_inertials.xml
  ```
  
- Launch the FT-IMU logger device,
  ```
  YARP_ROBOT_NAME=iCubGenova04 yarprobotinterface --config launch-ft-imu-logger.xml
  ```

- Press Ctrl+c to close the device, and the dataset is stored as the device is closed.

 Each dataset contains,

 - a `time` vector with the receive time stamps.
 - a struct for each strain-2 sensor attached on the robot (`l_foot_ft_imu`, `r_foot_ft_imu`, `r_leg_ft_imu`, `l_leg_ft_imu`) with each struct containing the corresponding FT sensor measurement, accelerometer, gyroscope and orientation sensor measurements.

Associated sensors,
- `l_leg_ft_imu`
    - FT: `l_leg_ft_sensor`
    - acc: `l_upper_leg_ft_acc_3b12`
    - gyro: `l_upper_leg_ft_gyro_3b12`
    - orientation: `l_upper_leg_ft_eul_3b12`
- `l_foot_ft_imu`
    - FT: `l_foot_ft_sensor`
    - acc: `l_foot_ft_acc_3b13`
    - gyro: `l_foot_ft_gyro_3b13`
    - orientation: `l_foot_ft_eul_3b13`
- `r_leg_ft_imu`
    - FT: `r_leg_ft_sensor`
    - acc: `r_upper_leg_ft_acc_3b11`
    - gyro: `r_upper_leg_ft_gyro_3b11`
    - orientation: `r_upper_leg_ft_eul_3b11`
- `r_foot_ft_imu`
    - FT: `r_foot_ft_sensor`
    - acc: `r_foot_ft_acc_3b14`
    - gyro: `r_foot_ft_gyro_3b14`
    - orientation: `r_foot_ft_eul_3b14`