# blf-calibration-delta-updater

**blf-calibration-delta-updater** is a simple tool to semi-automatically update the calibration delta of a YARP-based robot.

## :computer: Dependencies

**blf-calibration-delta-updater** depends on the [`YarpImplementation` of the `ISensorBridge`](https://github.com/dic-iit/bipedal-locomotion-framework/tree/master/src/RobotInterface/YarpImplementation) and on the [python bindings](https://github.com/dic-iit/bipedal-locomotion-framework/tree/master/bindings/python/RobotInterface). To run the script you also need to install some additional python dependencies

```
sudo apt-get install python3-numpy python3-lxml
```

## :running: How to use the application

The syntax of the application follows:

```shell
blf-calibration-delta-updater.py [-h] -i INPUT -o OUTPUT -p PART [--config CONFIG]
```

where:
- **`INPUT`** is the path to the input `xml` file containing the calibration deltas
- **`OUTPUT`** is the path to the output `xml` file containing the calibration deltas
- **`PART`** is the name of the group will be loaded in the configuration file. For instance [here](./config/robots/iCubGenova09/blf-calibration-delta-updater-options.ini) only `left_leg` or `right_leg` are admissible.
- **`CONFIG`** (optional) is the path to the configuration file loaded by the application. By default the **blf-calibration-delta-updater** uses [`YARP ResourceFinder`](http://www.yarp.it/git-master/resource_finder_spec.html) to locate a file named `blf-calibration-delta-updater-options.ini`

The application will ask you which joints you want to calibrate and will automatically create a configuration file.
