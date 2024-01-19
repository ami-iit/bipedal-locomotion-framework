# joints-grid-position-tracking
The **joints-grid-position-tracking** is an application that allows to move the joints
through all the combinations of a set of way points.

## 🏃 How to use the application
The fastest way to use the utility is to run the `python` application
[`blf-joints-grid-position-tracking.py`](./blf-joints-grid-position-tracking.py).
If you correctly installed the framework, you can run the application from any folder.

The application will:
1. Move the robot joints following a trajectory computed to pass through all the way points specified in
   [blf-joints-grid-position-tracking-options.ini](./config/robots/ergoCubGazeboV1_1/blf-joints-grid-position-tracking-options.ini)
   ```ini
   r_hip_pitch       (-0.2, 0.44)
   r_hip_roll        (0.36, 0.14)
   ```
   The above lists represent the knots for the controlled robot joints.
   The first joint that is moved is the last in the list of the controlled joints.
   All the possible combinations are computed from the list of knots. For example, if you specify
   $3$ way points for $2$ joints you will have $9$ list of knots.
   The trajectory is generated using a cubic spline to guarantee null joint accelerations.
   The trajectory lasts `motion_duration` seconds. Once the knot is reached the planner
   will wait for `motion_timeout` seconds before starting a new trajectory.
2. Open a port named `/joints_grid_position_tracking/logger/data:o` containing the joint trajectory values structured as
   [`VectorCollection`](../../src/YarpUtilities/thrifts/BipedalLocomotion/YarpUtilities/VectorsCollection.thrift)
   data. The user may collect the data via [`YarpRobotLoggerDevice`](../../devices/YarpRobotLoggerDevice).

If you want to run the application for a different robot remember to create a new folder in
[`./config/robots/`](./config/robots). The name of the folder should match the name of the robot.
