/**
 * @file QuitHandler.cpp
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <csignal>
#include <cstring>
#include <thread>

#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace System
{
std::function<void()> customHandlerLambda;

void my_handler(int sig)
{
    static int ct = 0;

    if (sig == SIGABRT)
    {
        log()->info("Aborted.");
        // to avoid that std::abort is called again
        if (ct > 3)
        {
            return;
        }
    }

    ct++;
    if (ct > 3)
    {
        log()->info("Aborting (calling abort())...");
        std::abort();
    }
    log()->info("[try {} of 3] Trying to shut down.", ct);

    customHandlerLambda();
}

#ifdef _WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType)
    {
    // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleQuitSignals(std::function<void()> customHandler)
{
#ifdef _WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
    customHandlerLambda = customHandler;
}

} // namespace System
} // namespace BipedalLocomotion
