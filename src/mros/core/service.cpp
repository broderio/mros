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
        int status = publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Service failed to receive message");
            return;
        }
        else if (SOCKET_STATUS_IS_WARNING(status))
        {
            return;
        }

        // Invoke callback
        std::string outMsg;
        callbackHelper->invokeCallback(inMsg, outMsg);

        // Send response
        status = publicServer.sendTo(outMsg, sender);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Service failed to send response");
        }
    }
}