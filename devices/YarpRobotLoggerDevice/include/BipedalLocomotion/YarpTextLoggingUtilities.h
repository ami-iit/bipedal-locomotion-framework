/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H

#include <cstdint>
#include <string>

#include <yarp/os/Bottle.h>

namespace BipedalLocomotion
{
/**
 * Structure representing the TextLoggingEntry
 * @note The code related to this structure has been taken from
 * https://github.com/robotology/yarp/blob/1986858acb14880bccf9073e3e74c082dc5caa0b/src/libYARP_logger/src/yarp/logger/YarpLogger.h#L121-L141
 */
struct TextLoggingEntry
{
    bool isValid{false};
    std::string level;
    std::string text;
    std::string filename;
    unsigned int line{0};
    std::string function;
    std::string hostname;
    std::string cmd;
    std::string args;
    int pid{0};
    std::int64_t thread_id{0};
    std::string component;
    std::string id;
    double systemtime{0.0};
    double networktime{0.0};
    double externaltime{0.0};
    std::string backtrace;
    std::string yarprun_timestamp;
    std::string local_timestamp;

    std::string portComplete;
    std::string portSystem;
    std::string portPrefix;
    std::string processName;
    std::string processPID;

    /**
     * Demoralize a text logging message,
     * @note The code has been taken from
     * https://github.com/robotology/yarp/blob/1986858acb14880bccf9073e3e74c082dc5caa0b/src/libYARP_logger/src/yarp/logger/YarpLogger.h#L121-L141
     */
    static TextLoggingEntry
    deserializeMessage(const yarp::os::Bottle& message, const std::string& currentTime);
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_YARP_TEXT_LOGGING_UTILITIES_H
