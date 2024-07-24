#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <csignal>

namespace mros
{

    class SignalHandler
    {
    public:
        SignalHandler();

        bool ok();

    private:
        static void handle(int signal);

        static bool shutdown;
    };

} // namespace mros