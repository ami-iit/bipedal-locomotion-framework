/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H

#include <string>

#include <yarp/os/Bottle.h>

namespace BipedalLocomotion
{

struct TextLoggingEntry
{
    bool isValid{false};
    std::string level;
    std::string text;
    std::string filename;
    unsigned int line;
    std::string function;
    std::string hostname;
    std::string cmd;
    std::string args;
    int pid;
    long thread_id;
    std::string component;
    std::string id;
    double systemtime;
    double networktime;
    double externaltime;
    std::string backtrace;
    std::string yarprun_timestamp;
    std::string local_timestamp;

    std::string portComplete;
    std::string portSystem;
    std::string portPrefix;
    std::string processName;
    std::string processPID;

    static TextLoggingEntry
    deserializeMessage(const yarp::os::Bottle& message, const std::string& currentTime);
};


} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H
