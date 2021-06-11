# calibration-delta-updater

**calibration-delta-updater** is a simple tool for updating the calibration delta of the robot.

## :computer: Dependencies

**calibration-delta-updater** depends of the [`YarpImplementation` of the `ISensorBridge`](https://github.com/dic-iit/bipedal-locomotion-framework/tree/master/src/RobotInterface/YarpImplementation) and on the [python bindings](https://github.com/dic-iit/bipedal-locomotion-framework/tree/master/bindings/python/RobotInterface). To run the script you also need to install some additional python dependencies

```
sudo apt-get install python3-numpy python3-lxml
```

## :running: How to use the application

Please move the joint associated to the `input` configuration file in _zero_. Please refer to the posture shown [here](http://wiki.icub.org/wiki/LegsFineCalibration).

You can run the script with following command
```shell
blf_calibration_delta_updater.py -i <robots-configuration_dir>/iCubGenova09/calibrators/left_leg-calib.xml \
                                 -o <robots-configuration_dir>/iCubGenova09/calibrators/left_leg-calib.xml \
                                 -r icub \
                                 -b left_leg \
                                 -j l_hip_pitch l_hip_roll l_hip_yaw l_knee l_ankle_pitch l_ankle_roll
```
