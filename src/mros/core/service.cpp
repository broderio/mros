#include "mros/core/service.hpp"

namespace mros
{

    Service::~Service()
    {
        shutdown();
        publicServer.close();
    }

    std::string Service::getService() const
    {
        return service;
    }

    void Service::shutdown()
    {
        if (shutdownFlag)
        {
            return;
        }
        shutdownFlag = true;
    }

    void Service::runOnce()
    {
        // Check publicServer for incoming messages
        std::string inMsg;
        URI sender;
        if (publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender) < 0)
        {
            Console::log(LogLevel::ERROR, "Service failed to receive message");
            return;
        }

        if (inMsg.empty())
        {
            return;
        }

        // Invoke callback
        std::string outMsg;
        callbackHelper->invokeCallback(inMsg, outMsg);

        // Send response
        if (publicServer.sendTo(outMsg, sender) < 0)
        {
            Console::log(LogLevel::ERROR, "Service failed to send response");
        }
    }
}