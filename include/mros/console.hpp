#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <mutex>
#include "stdlib.h"

#include "utils.hpp"

namespace mros
{
    enum LogLevel
    {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    class Console
    {
    public:
        ~Console();

        static void init(const std::string &name, bool logToFile = false);
        static void log(LogLevel level, const std::string &msg);

    private:

        static bool initialized;

        static bool logToFile;

        static std::string name;

        const static std::string defaultColor; // Reset color

        static int64_t startTime;

        static std::mutex mutex;

        static std::ofstream logFile;

        static std::string getLevelString(LogLevel level);

        static std::string getLevelColor(LogLevel level);
    };
} // namespace mros