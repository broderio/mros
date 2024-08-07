#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include "socket/common.hpp"
#include "socket/udp/server.hpp"

#include "utils.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"
#include "mros/utils/callback.hpp"
#include "mros/utils/signalHandler.hpp"

namespace mros
{
    
    class Node;


    class Service
    {

        friend class Node;

    public:

        ~Service();

        std::string getService() const;

        void shutdown();

    private:

        template <typename InMsgType, typename OutMsgType>
        Service(const std::string &service, std::function<void(const InMsgType &request, OutMsgType &response)> callback)
            : publicServer(URI(getLocalIP(), 0))
        {
            static_assert(std::is_base_of<IMessage, InMsgType>::value, "InMsgType must inherit from IMessage");
            static_assert(std::is_base_of<IMessage, OutMsgType>::value, "OutMsgType must inherit from IMessage");

            this->service = service;
            this->inMsgType = typeid(InMsgType).name();
            this->outMsgType = typeid(OutMsgType).name();
            this->shutdownFlag = false;
            this->callbackHelper = std::make_shared<SrvCallbackHelper<InMsgType, OutMsgType>>(callback);

            publicServer.bind();
        }

        template <typename InMsgType, typename OutMsgType>
        static std::shared_ptr<Service> create(const std::string &service, std::function<void(const InMsgType &request, OutMsgType &response)> callback)
        {
            std::shared_ptr<Service> server(new Service(service, callback));
            return server;
        }

        void runOnce();

        std::string service;
        std::string inMsgType;
        std::string outMsgType;
        bool shutdownFlag;

        std::shared_ptr<SrvCallbackHelperBase> callbackHelper;

        UDPServer publicServer;
    };
}