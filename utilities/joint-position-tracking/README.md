# joint-position-tracking

The **joint-position-tracking** is a simple tool for testing the joint position tracking of a robot in a `YARP` environment.

## :running: How to use the application
The fastest way to use the application is to run the `sh` script [`blf-joint-position-tracking-script.sh`](./script/blf-joint-position-tracking-script.sh). If you correctly installed the the framework you can run the script from any folder.

The script will:
- move the robot
- store the desired and actual joint position in a `txt` file
- run a python script to plot the data in a file called `figure.png`

The [`blf-joint-position-tracking-options.ini`](./config/robots/iCubGenova09/blf-joint-position-tracking-options.ini) file contains some parameters that you may modify to control a given joint:
- `max_angle_deg`: is the max angle reached by the joint (the value is in degrees)
- `min_angle_def`: is the min angle reached by the joint (the value is in degrees)
- `trajectory_duration`: is the duration of the generated trajectory (the value is in seconds)
- `robot_name`: name of the robot
- `joints_list`: list of the controlled joint (for the time being multiple joints control is not supported. Please specify only one joint)
- `remote_control_boards`: list of associated control board (for the time being multiple joints control is not supported. Please specify only one control board)

Please if you want to run the application for a different robot remember to create a new folder in `./config/robots/`. The name of the folder should match the name of the robot.


This is an example of usage

![](https://user-images.githubusercontent.com/16744101/97494667-e5576280-1966-11eb-840b-56e5120f6b29.gif)
