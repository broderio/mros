#include "mros/utils/signalHandler.hpp"

namespace mros
{

    bool SignalHandler::shutdown = false;

    std::function<void(int)> SignalHandler::handler = nullptr;

    void SignalHandler::init(std::function<void(int)> handler)
    {
        SignalHandler::handler = handler;
        signal(SIGINT, SignalHandler::handle);
    }

    bool SignalHandler::ok() { return !shutdown; }

    void SignalHandler::handle(int signal)
    {
        SignalHandler::shutdown = true;

        if (SignalHandler::handler != nullptr)
        {
            SignalHandler::handler(signal);
        }
    }

} // namespace mros