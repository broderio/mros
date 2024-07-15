#include "mros/console.hpp"

namespace mros
{

    bool Console::initialized = false;
    std::string Console::name;
    const std::string Console::defaultColor = "\033[0m"; // Reset color
    int64_t Console::startTime;
    std::mutex Console::mutex;
    std::ofstream Console::logFile;

    void Console::init(const std::string &name)
    {
        if (initialized)
        {
            return;
        }
        initialized = true;

        Console::name = name;

        DateTime dt = getDateTime();
        system(std::string("mkdir -p ~/.mros/"+name).c_str());

        std::string homeDir = getenv("HOME");
        logFile.open(homeDir + "/.mros/" + name + "/" + dt.toString() + ".log");
        startTime = getTimeNano();
    }

    void Console::log(LogLevel level, const std::string &msg)
    { 
        // Format: [HH:MM:SS] [LEVEL] [NAME] MESSAGE
        int64_t ns_total = getTimeNano() - startTime;

        int64_t s = ns_total / 1000000000;
        int64_t ms = (ns_total / 1000000) % 1000;
        int64_t ns = ns_total % 1000000;

        std::string time = "[" + std::to_string(s) + ":" + std::to_string(ms) + ":" + std::to_string(ns) + "]";
        std::string levelString = getLevelString(level);
        std::string color = getLevelColor(level);

        std::lock_guard<std::mutex> lock(mutex);
        if (level > LogLevel::INFO)
        {
            std::cerr << time << "\t[" << color << levelString << Console::defaultColor << "]\t[" << name << "]\t" << msg << '\n';
        }
        else {
            std::cout << time << "\t[" << color << levelString << Console::defaultColor << "]\t[" << name << "]\t" << msg << '\n';
        }

        logFile << time << "\t[" << levelString << "]\t[" << name << "]\t" << msg << '\n';
    }

    std::string Console::getLevelString(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARN:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
        }
    }

    std::string Console::getLevelColor(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::DEBUG:
            return "\033[1;34m"; // Blue
        case LogLevel::INFO:
            return "\033[1;32m"; // Green
        case LogLevel::WARN:
            return "\033[1;33m"; // Yellow
        case LogLevel::ERROR:
            return "\033[1;31m"; // Red
        case LogLevel::FATAL:
            return "\033[1;35m"; // Purple
        default:
            return "\033[1;37m"; // White
        }
    }

} // namespace mros