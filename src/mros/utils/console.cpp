#include "mros/utils/console.hpp"

namespace mros
{

    bool Console::initialized = false;
    std::string Console::name;
    LogLevel Console::level;
    const std::string Console::defaultColor = "\033[0m"; // Reset color
    int64_t Console::startTime;
    std::mutex Console::mutex;
    std::ofstream Console::logFile;
    bool Console::logToFile = false;

    void Console::init(const std::string &name, LogLevel level, bool logToFile)
    {
        if (initialized)
        {
            Console::log(LogLevel::WARN, "Console already initialized");
            return;
        }
        initialized = true;

        Console::name = name;

        Console::level = level;

        DateTime dt = getDateTime();

        startTime = getTimeNano();

        Console::logToFile = logToFile;
        if (!logToFile)
        {
            return;
        }
        system(std::string("mkdir -p ~/.mros/"+name).c_str());
        std::string homeDir = getenv("HOME");
        logFile.open(homeDir + "/.mros/" + name + "/" + dt.toString() + ".log");
    }

    void Console::log(LogLevel level, const std::string &msg)
    { 
        if (!initialized)
        {
            std::cerr << "Console not initialized!\n";
            return;
        }

        if (level < Console::level)
        {
            return;
        }

        // Format: [HH:MM:SS] [LEVEL] [NAME] MESSAGE
        int64_t ns_total = getTimeNano() - startTime;

        int64_t s = ns_total / 1000000000;
        int64_t ms = (ns_total / 1000000) % 1000;
        int64_t ns = ns_total % 1000000;

        std::string time = "[" + std::to_string(s) + ":" + std::to_string(ms) + ":" + std::to_string(ns) + "] ";
        std::string levelString = "[" + getLevelColor(level) + getLevelString(level) + Console::defaultColor + "] ";
        std::string nameStr = "[" + name + "] ";

        std::lock_guard<std::mutex> lock(mutex);
        if (level > LogLevel::INFO)
        {
            std::cerr << std::left << std::setw(20) << time
                      << std::left << std::setw(8) << levelString
                      << std::left << std::setw(26) << nameStr
                      << msg
                      << std::endl;
        }
        else {
            std::cout << std::left << std::setw(20) << time
                      << std::left << std::setw(8) << levelString
                      << std::left << std::setw(26) << nameStr
                      << msg
                      << std::endl;
        }

        if (Console::logToFile)
        {
            logFile << std::left << std::setw(20) << time
                    << std::left << std::setw(8) << levelString
                    << std::left << std::setw(26) << nameStr
                    << msg
                    << std::endl;
        }
    }

    void Console::setLevel(LogLevel level)
    {
        if (!initialized)
        {
            std::cerr << "Console not initialized!\n";
            return;
        }
        
        Console::level = level;
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