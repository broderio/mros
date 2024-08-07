#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <functional>

namespace mros
{

    class SignalHandler
    {
    public:
        SignalHandler() = delete;

        static void init(std::function<void(int)> handler = nullptr);

        static bool ok();

    private:
        static void handle(int signal);

        static std::function<void(int)> handler;

        static bool shutdown;
    };

} // namespace mros