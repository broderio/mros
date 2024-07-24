#pragma once

#include <string>
#include <functional>

#include "messages/message.hpp"

namespace mros
{
    class SubCallbackHelperBase
    {
    public:
        virtual void invokeCallback(const std::string &msg) = 0;
    };

    template <typename MsgType>
    class SubCallbackHelper : public SubCallbackHelperBase
    {
    public:
        SubCallbackHelper(std::function<void(const MsgType &)> callback)
            : callback(callback) {}

        void invokeCallback(const std::string &msg) override
        {
            MsgType m;
            Parser::decode(msg, m);
            callback(m);
        }

    private:
        std::function<void(const MsgType &)> callback;
    };

    class SrvCallbackHelperBase
    {
    public:
        virtual void invokeCallback(const std::string &req, std::string &res) = 0;
    };

    template <typename InMsgType, typename OutMsgType>
    class SrvCallbackHelper : public SrvCallbackHelperBase
    {
    public:
        SrvCallbackHelper(std::function<void(const InMsgType &, OutMsgType &)> callback)
            : callback(callback) {}

        void invokeCallback(const std::string &req, std::string &res) override
        {
            InMsgType in;
            OutMsgType out;
            Parser::decode(req, in);
            OutMsgType msg;
            callback(in, msg);
            res = Parser::encode(msg, 0);
        }

    private:
        std::function<void(const InMsgType &, OutMsgType &)> callback;
    };
}