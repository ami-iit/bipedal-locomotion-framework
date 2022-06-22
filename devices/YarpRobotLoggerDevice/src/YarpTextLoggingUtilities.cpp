/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <string>

#include <BipedalLocomotion/YarpTextLoggingUtilities.h>
#include <BipedalLocomotion/System/Clock.h>

BipedalLocomotion::TextLoggingEntry
BipedalLocomotion::TextLoggingEntry::deserializeMessage(const yarp::os::Bottle& message, const std::string& currentTime)
{
    if (message.size() != 2)
    {
        return BipedalLocomotion::TextLoggingEntry();
    }

    std::string bottlestring = message.toString();
    std::string header;

    if (message.get(0).isString())
    {
        header = message.get(0).asString();
    } else
    {
        fprintf(stderr, "ERROR: unknown log format!\n");
        return BipedalLocomotion::TextLoggingEntry();
    }

    BipedalLocomotion::TextLoggingEntry body;

    // TODO
    // char ttstr[20];
    // static int count = 0;
    // sprintf(ttstr, "%d", count++);
    // body.yarprun_timestamp = std::string(ttstr);
    body.local_timestamp = currentTime;

    std::string s;
    if (message.get(1).isString())
    {
        s = message.get(1).asString();
    } else
    {
        return BipedalLocomotion::TextLoggingEntry();
    }

    yarp::os::Property p(s.c_str());

    if (p.check("level"))
    {
        body.text = p.find("message").toString();
        body.level = p.find("level").toString();

        if (p.check("filename"))
        {
            body.filename = p.find("filename").asString();
        } else
        {
            body.filename.clear();
        }

        if (p.check("line"))
        {
            body.line = static_cast<uint32_t>(p.find("line").asInt32());
        } else
        {
            body.line = 0;
        }

        if (p.check("function"))
        {
            body.function = p.find("function").asString();
        } else
        {
            body.function.clear();
        }

        if (p.check("hostname"))
        {
            body.hostname = p.find("hostname").asString();
        } else
        {
            body.hostname.clear();
        }

        if (p.check("pid"))
        {
            body.pid = p.find("pid").asInt32();
        } else
        {
            body.pid = 0;
        }

        if (p.check("cmd"))
        {
            body.cmd = p.find("cmd").asString();
        } else
        {
            body.cmd.clear();
        }

        if (p.check("args"))
        {
            body.args = p.find("args").asString();
        } else
        {
            body.args.clear();
        }

        if (p.check("thread_id"))
        {
            body.thread_id = p.find("thread_id").asInt64();
        } else
        {
            body.thread_id = 0;
        }

        if (p.check("component"))
        {
            body.component = p.find("component").asString();
        } else
        {
            body.component.clear();
        }

        if (p.check("id"))
        {
            body.id = p.find("id").asString();
        } else
        {
            body.id.clear();
        }

        if (p.check("systemtime"))
        {
            body.systemtime = p.find("systemtime").asFloat64();
        } else
        {
            body.systemtime = 0.0;
        }

        if (p.check("networktime"))
        {
            body.networktime = p.find("networktime").asFloat64();
        } else
        {
            body.networktime = body.systemtime;
            body.yarprun_timestamp.clear();
        }

        if (p.check("externaltime"))
        {
            body.externaltime = p.find("externaltime").asFloat64();
        } else
        {
            body.externaltime = 0.0;
        }

        if (p.check("backtrace"))
        {
            body.backtrace = p.find("backtrace").asString();
        } else
        {
            body.backtrace.clear();
        }
    } else
    {
        // This is plain output forwarded by yarprun
        // Perhaps at some point yarprun could be formatting it properly
        // But for now we just try to extract the level information
        body.text = s;
        body.level = "LOGLEVEL_UNDEFINED";

        size_t str = s.find('[', 0);
        size_t end = s.find(']', 0);
        if (str == std::string::npos || end == std::string::npos)
        {
            body.level = "LOGLEVEL_UNDEFINED";
        } else if (str == 0)
        {
            std::string level = s.substr(str, end + 1);
            body.level = "LOGLEVEL_UNDEFINED";
            if (level.find("TRACE") != std::string::npos)
            {
                body.level = "LOGLEVEL_TRACE";
            } else if (level.find("DEBUG") != std::string::npos)
            {
                body.level = "LOGLEVEL_DEBUG";
            } else if (level.find("INFO") != std::string::npos)
            {
                body.level = "LOGLEVEL_INFO";
            } else if (level.find("WARNING") != std::string::npos)
            {
                body.level = "LOGLEVEL_WARNING";
            } else if (level.find("ERROR") != std::string::npos)
            {
                body.level = "LOGLEVEL_ERROR";
            } else if (level.find("FATAL") != std::string::npos)
            {
                body.level = "LOGLEVEL_FATAL";
            }
            body.text = s.substr(end + 1);
        } else
        {
            body.level = "LOGLEVEL_UNDEFINED";
        }
    }

    std::istringstream iss(header);
    std::string token;
    getline(iss, token, '/');
    getline(iss, token, '/');
    body.portSystem = token;
    getline(iss, token, '/');
    body.portPrefix = token;
    getline(iss, token, '/');
    body.processName = token;
    getline(iss, token, '/');
    body.processPID = token.erase(token.size() - 1);

    body.isValid = true;

    return body;
}
