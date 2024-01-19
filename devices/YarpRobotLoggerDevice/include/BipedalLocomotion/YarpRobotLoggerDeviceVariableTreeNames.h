#ifndef BIPEDAL_LOCOMOTION_fRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_VTN_H
#include <iostream>

#define BIPEDAL_LOCOMOTION_fRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_VTN_H

#define TREE_DELIM                              "::"

#define ROBOT_RT_ROOT_NAME                      "robot_realtime"


#define JOINT_STATE_POSITIONS_NAME              "joints_state::positions"
#define JOINT_STATE_VELOCITIES_NAME             "joints_state::velocities"
#define JOINT_STATE_ACCLERATIONS_NAME           "joints_state::accelerations"
#define JOINT_STATE_TORQUES_NAME                "joints_state::torques"

#define MOTOR_STATE_POSITIONS_NAME              "motors_state::positions"
#define MOTOR_STATE_VELOCITIES_NAME             "motors_state::velocities"
#define MOTOR_STATE_ACCELERATIONS_NAME          "motors_state::acclerations"
#define MOTOR_STATE_CURRENTS_NAME               "motors_state::currents"
#define MOTOR_STATE_PWM_NAME                    "motors_state::PWM"

#define MOTOR_STATE_PIDS_NAME                   "PIDs"

#define FTS_NAME                                "FTs"

const std::string FTElementNames[] =             {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"};

#define GYROS_NAME                                "gyros"
const std::string GyroElementNames[] =            {"omega_x", "omega_y", "omega_z"};

#define ACCELEROMETERS_NAME                       "accelerometers"
const std::string AccelerometerElementNames[] =   {"a_x", "a_y", "a_z"};

#define ORIENTATIONS_NAME                         "orientations"
const std::string OrientationElementNames[] =     {"r", "p", "y"};

#define MAGNETOMETERS_NAME                        "magnetometers"
const std::string MagnetometerElementNames[] =    {"mag_x", "mag_y", "mag_z"};

#define CARTESIAN_WRENCHES_NAME                   "cartesian_wrenches"
const std::string CartesianWrenchNames[] =        {FTElementNames[0], FTElementNames[1], FTElementNames[2], FTElementNames[3], FTElementNames[4], FTElementNames[5]};

#define TEMPERATURE_NAME                          "temperatures"
const std::string TemperatureNames[] =            {"temperature"};

#define YARP_NAME                                 "yarp_robot_name"

#define ROBOT_DESCRIPTON_LIST                     "description_list"

#define TIMESTAMPS_NAME                           "timestamps"

#endif




