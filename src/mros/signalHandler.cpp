#include "mros/signalHandler.hpp"

namespace mros
{

    bool SignalHandler::shutdown = false;

    SignalHandler::SignalHandler()
    {
        signal(SIGINT, SignalHandler::handle);
    }

    bool SignalHandler::ok() { return !shutdown; }

    void SignalHandler::handle(int signal)
    {
        SignalHandler::shutdown = true;
    }

} // namespace mros