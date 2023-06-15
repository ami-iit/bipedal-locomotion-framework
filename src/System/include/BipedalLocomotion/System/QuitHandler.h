/**
 * @file QuitHandler.h
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_QUIT_HANDLER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_QUIT_HANDLER_H

#include <functional>

namespace BipedalLocomotion
{

namespace System
{

/**
 * handleQuitSignals should be called if you want to call a function when ctrl-c event is detected.
 * @param customHandler function that will be called when ctrl-c event is detected
 * @note Please check the following example if you want to use the function
 * \code{.cpp}
 * #include <iostream>
 * #include <chrono>
 * #include <thread>
 *
 * #include <BipedalLocomotion/System/QuitHandler.h>
 *
 * int main()
 * {
 *     // When ctrl-c event is called the application will print "Closing..." in the terminal.
 *     BipedalLocomotion::System::handleQuitSignals([](){std::cout << "Closing..." << std::endl;});
 *
 *     while(true)
 *         std::this_thread::sleep_for(std::chrono::milliseconds(100));
 *
 *     return EXIT_SUCCESS;
 * }
 * \endcode
 * @note The original implementation can be found here:
 * https://github.com/robotology/idyntree-yarp-tools/blob/main/src/lib/Utilities/Utilities.h
 */
void handleQuitSignals(std::function<void()> customHandler = std::function<void()>());

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_QUIT_HANDLER_H
