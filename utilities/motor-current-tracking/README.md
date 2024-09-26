# motor-current-tracking
The **motor-current-tracking** is an application that allows to send current commands
to the motors. The user can leverage predefined signals to define the reference current.

## 🏃 How to use the application
It is a command line application, which can be run like:

```sh
blf-motor-current-tracking.py
```

The application accepts an additional argument to specify the type of trajectory to deploy.
From terminal, run:

```sh
blf-motor-current-tracking.py -h
```

to display the help message which details how to pass the additional input argument.

If bipedal locmotion framework is installed correctly, the application can be run from anywhere. In order to install the application, you should set the option `FRAMEWORK_COMPILE_MotorCurrentTrackingApplication` to `ON`.

Once run, the application will:

1. Generate the current trajectory used to drive the motors.

2. Drive the motors in current as long as they are within the safety limits, which are defined in the configuration files.

In fact, the application will check if the joints come close to the position limits. In this case, the related motors are switch to position control
and kept still in the last read position as long as all the other motors have completed the current trajectory.

Moreover, the application streams data, which is the motors reference current and the control mode, through the port
`/motor_current_tracking/logger/data:o`. The user may collect the data via [`YarpRobotLoggerDevice`](../../devices/YarpRobotLoggerDevice).

## Configuration files
Configuration files allow to specify the motors to drive and the current trajectory to use.
[`blf-motor-current-tracking-options.ini`](./config/robots/ergoCubSN001/blf-motor-current-tracking-options.ini) is an example of the main configuration file.

In this file, the following parameters are defined:

1. the sample time `dt` of the reference trajectory
2. the joints position safety threshold `safety_threshold`, which restricts the motion range of joints to `[joint_lower_limit + safety_threshold, joint_upper_limit - safety_threshold]`
3. the `use_safety_threshold_for_starting_position` boolean, which determines wheter to use (or not) the `safety_threshold` when computing the starting positions.
4. the number of starting positions `number_of_starting_points` from which the same reference trajectory is repeated.
5. the `SINUSOID` parameters group, which defines the sinusoid trajectory as detailed later
6. the `RAMP` parameters group, which defines the ramp trajectory as detailed later

Instead, the configuration file [`robot_control.ini`](./config/robots/ergoCubSN001/blf_motor_current_tracking/robot_control.ini) defines the list of motors to command.

If you want to run the application for a different robot remember to create a new folder in
[`./config/robots/`](./config/robots). The name of the folder should match the name of the robot.

## Predefined Trajectories
Two trajectory types are currently supported: sinusoids and ramps.

### Sinusoids

They are defined as sinusoidal signals with varying frequency `f` and amplitude `A`:

```math

current = current_0 + A \sin (2 \pi f t)

```

The frequency is gradually decreased, every 2 cycles, from `max_frequency` to `min_frequency` by `frequency_decrement`.
Once the frequency range is covered the amplitude is increased by `delta_current_increment`.
The starting amplitude is set by `min_delta_current`, while the final one by `max_delta_current`.

Note that for some motors, such as the `ankles` ones, the initial current `current_0` is not the measured one at the starting position, but is set to `0`.

Each current sinusoidal signal is repeated for each starting position.

These starting positions are computed by dividing the range of motion of the joint
(`[joint_lower_limit, joint_upper_limit]`) in `number_of_starting_points + 1` segments, and
then taking the inner nodes.

Finally, every parameter of the `SINUSOID` group has to be defined by a vector whose length corresponds to the number
of motors to drive.


### Ramps

They are ramp signals which increases of from an initial current to a maximum current, with a slope that ranges from `initial_speed` to `final_speed`.

Depending on the motors, the initial current is set to `0` or to the mesured one at the starting position.
While the maximum current is given by the initial current increased of `max_current_increment`.

Note that, each ramp is repeated twice for each starting position, in order to cover both direction of motion.
Morever from one ramp to the following one the slope is incremented by `speed_increment`.

The sarting positions are computed like for the sinusoidal trajectories.

Finally, every parameter of the `RAMP` group has to be defined by a vector whose length corresponds to the number
of motors to drive.

### Examples

The following are some set of configurations which were tested on `ergoCubSN001`.

```
dt 0.002 #[s]
safety_threshold 2.0 #[deg]
use_safety_threshold_for_starting_position false
number_of_starting_points 5

[SINUSOID]
min_delta_current             ( 0.7  ) #[A]
max_delta_current             ( 1.8  ) #[A]
delta_current_increment       ( 0.2  ) #[A]
min_frequency                 ( 0.05 ) #[Hz]
max_frequency                 ( 0.45 ) #[Hz]
frequency_decrement           ( 0.1  ) #[Hz]

[RAMP]
initial_speed                    (0.15)
final_speed                      (0.40)
speed_increment                  (0.08)
max_current_increment            (2.00)

[ROBOT_CONTROL]
robot_name                              ergocub
joints_list                             ("r_ankle_pitch")
remote_control_boards                   ("right_leg")
positioning_duration                    3.0  #[s]
positioning_tolerance                   0.05 #[s]
position_direct_max_admissible_error    0.1  #[s]
```
