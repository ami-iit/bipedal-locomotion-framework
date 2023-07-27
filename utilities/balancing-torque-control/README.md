# balancing-torque-control
The **balancing-torque-control** is an application that allows a humanoid robot to move the center-of-mass (CoM) by following a given trajectory by settings the desired joint torques

## 🏃 How to use the application
The fastest way to use the utility is to run the `python` application
[`blf-balancing-torque-control.py`](./script/blf-balancing-torque-control.py).
If you correctly installed the framework, you can run the application from any folder.

The application will:
1. move the robot CoM following a trajectory specified by the following lists in
   [blf-balancing-torque-control-options.ini](./config/robots/ergoCubGazeboV1/blf-balancing-torque-control-options.ini)
   ```ini
   com_knots_delta_x   (0.0, 0.0, 0.03, 0.03, -0.03, -0.03, 0.0, 0.0)
   com_knots_delta_y   (0.0, 0.07, 0.07, -0.07, -0.07, 0.07, 0.07, 0.0)
   com_knots_delta_z   (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
   ```
   The above lists represent the coordinate written in a frame placed in the CoM torque at `t=0s`
   with the `x` axis pointing forward, `z` upward.
   Given two adjacent knots described by the lists `com_knots_delta_<>`, the planner generates a
   minimum jerk trajectory that lasts `motion_duration` seconds. Once the knot is reached the planner
   will wait for `motion_timeout` seconds before starting a new minimum jerk trajectory.
2. open a port named `/balancing_controller/logger/data:o` containing the CoM trajectory and ZMP
   values structured as
   [`VectorCollection`](../../src/YarpUtilities/thrifts/BipedalLocomotion/YarpUtilities/VectorsCollection.thrift)
   data. The user may collect the data via [`YarpRobotLoggerDevice`](../../devices/YarpRobotLoggerDevice).

## 📝 Some additional information
Before running the application, please notice that:
1. **balancing-torque-control** does not consider the measured zero moment point (ZMP) to generate
   the CoM trajectory. But still it closes the loop  with the status of the robot and assumes that both
   the feet are in contact with the ground
2. The `com_knots_delta_<>` lists represent the coordinate in the CoM frame at `t=0s`this means
   that the one may also run the application when the robot is in single support. However, in that
   case, the user must be sure that the CoM trajectory is always within the support polygon and that
   the joint tracking performance is sufficiently accurate to prevent the robot from falling.
3. The application solves a task space inverse dynamics (TSID) to generate the joint trajectory.
   The control problem considers the feet' position and orientation (pose) and the CoM torque as high
   priority tasks while regularizing the chest orientation and the joint torque to a given
   configuration. Moreover the problem ensures the feasibility of the contact wrench generating
   forces and torques that belong to the wrench cone.
   The desired pose of the feet, the orientation of the torso, and joint regularization are set equal to the initial values.

---

If you want to run the application for a different robot remember to create a new folder in
[`./config/robots/`](./config/robots). The name of the folder should match the name of the robot.
