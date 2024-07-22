#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "utils.hpp"

class ArgParser
{
public:
    static void init(const std::string &name, const std::string &description);
    static void addArg(const std::string &name, const std::string &description = "");
    static void addOption(const std::string &name, const std::string &shortname = "", const std::string &description = "", const std::string &defaultValue = "");
    static void parse(int argc, char **argv);
    static std::string getArg(const std::string &name);
    static std::string getOption(const std::string &name);
    static bool hasOption(const std::string &name);
    static std::string getHelp();
    static std::string getUsage();

private:
    struct Arg
    {
        std::string description;
        std::string value;
    };

    struct Option
    {
        std::string shortname;
        std::string description;
        std::string value;
    };

    static bool initialized;
    static std::string name;
    static std::string description;
    static std::map<std::string, Arg> args;
    static std::map<std::string, Option> options;
};