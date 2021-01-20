# joint-trajectory-player

The **joint-trajectory-player** is a simple tool for playing a joint trajectory on a robot in a `YARP` environment. The tool allows you to save the measured joint positions and measured currents.

## :running: How to use the application
The application can be launched with the following command:
```
blf-joint-trajectory-player --from blf-joint-trajectory-player-options.ini --trajectory_file file_of_the_joint_trajectory.mat
```
The `.mat` file must have a field called `traj` containing the trajectory stored as a matrix in row major order.
If you correctly installed the framework you can run the application from any folder.

The [`blf-joint-trajectory-player-options.ini`](./config/robots/iCubGazeboV3/blf-joint-trajectory-player-options.ini) file contains some parameters that you may modify to control a given set of joints:
- `robot_name`: name of the robot
- `joints_list`: list of the controlled joints
- `remote_control_boards`: list of associated control boards

Please if you want to run the application for a different robot remember to create a new folder in `./config/robots/`. The name of the folder should match the name of the robot.
