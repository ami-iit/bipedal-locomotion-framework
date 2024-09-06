# balancing-torque-control

The **balancing-torque-control** application allows a humanoid robot to move its center-of-mass
(CoM) by following a given trajectory by setting the desired joint torques.


## 🏃 How to Use the Application

The fastest way to use the utility is to run the `python` application
[`blf-balancing-torque-control.py`](./script/blf-balancing-torque-control.py). If the framework is
correctly installed, you can run the application from any folder.

The application will:

1. Move the robot's CoM by following a trajectory specified in the lists found in
   [blf-balancing-torque-control-options.ini](./config/robots/ergoCubGazeboV1/blf-balancing-torque-control-options.ini):
   ```ini
   com_knots_delta_x   (0.0, 0.0, 0.03, 0.03, -0.03, -0.03, 0.0, 0.0)
   com_knots_delta_y   (0.0, 0.07, 0.07, -0.07, -0.07, 0.07, 0.07, 0.0)
   com_knots_delta_z   (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
   ```
   These lists represent coordinates in a frame located at the CoM at `t=0s`, with the `x` axis
   pointing forward and the `z` axis upward. Between adjacent knots described by the
   `com_knots_delta_<>` lists, the planner generates a minimum jerk trajectory that lasts
   `motion_duration` seconds. Once the knot is reached, the planner waits for `motion_timeout`
   seconds before starting a new minimum jerk trajectory.

2. Open a port named `/balancing_controller/logger/data:o` containing the CoM trajectory and ZMP
   values structured as
   [`VectorCollection`](../../src/YarpUtilities/thrifts/BipedalLocomotion/YarpUtilities/VectorsCollection.thrift)
   data. The user may collect this data via
   [`YarpRobotLoggerDevice`](../../devices/YarpRobotLoggerDevice).


## 📝 Additional Information

Before running the application, please note:

1. **balancing-torque-control** does not consider the measured zero moment point (ZMP) to generate
   the CoM trajectory. However, it still closes the loop using the robot's status and assumes both
   feet are in contact with the ground.

2. The `com_knots_delta_<>` lists represent the coordinates in the CoM frame at `t=0s`. This means
   that the application can also be run when the robot is in single support. However, in that case,
   the user must ensure that the CoM trajectory remains within the support polygon and that joint
   tracking performance is sufficiently accurate to prevent the robot from falling.

3. The application solves a task-space inverse dynamics (TSID) problem to generate the joint
   trajectory. The control problem prioritizes the feet's position and orientation (pose) and the
   CoM torque while regularizing the chest orientation and joint torque to a desired
   configuration. The problem also ensures that the contact wrenches are feasible, generating forces
   and torques within the wrench cone.
   The desired pose of the feet, torso orientation, and joint regularization are set to the initial
   values.

4. The list of controlled joints can be found in the configuration file
   [`robot_control.ini`](./config/robots/ergoCubGazeboV1/blf_balancing_torque_control/robot_control.ini).
   Removing a joint from the `joints_list` will exclude it from the control problem. The user must
   also adjust the `kp`, `kd`, and `weight` values in the `[JOINT_REGULARIZATION_TASK]` section to
   match the new joint list in
   [`tsid.ini`](./config/robots/ergoCubGazeboV1/blf_balancing_torque_control/tsid.ini). By default,
   if a joint is not in the `joints_list`, its position is considered to be zero. This behavior can
   be changed by setting the `fixed_joint_list_names` and `fixed_joint_list_values` in the
   [`robot_control.ini`](./config/robots/ergoCubGazeboV1/blf_balancing_torque_control/robot_control.ini)
   file. Specifically, `fixed_joint_list_names` is a list of joint names not in the `joints_list`,
   and `fixed_joint_list_values` contains the corresponding joint positions (in degrees).

---

If you want to run the application for a different robot, remember to create a new folder in
[`./config/robots/`](./config/robots). The folder name should match the name of the robot.
