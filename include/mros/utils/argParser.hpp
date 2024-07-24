#pragma once

#include <iostream>
#include <string>
#include <set>
#include <map>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "utils.hpp"

namespace mros {

class ArgParser
{
public:
    struct Opt
    {
        std::string name;
        std::string shortName;
        std::string description;
        std::string defaultVal; // value if not specified
        std::string constVal; // value if specified without argument
        char nargs;

        bool operator<(const Opt &other) const
        {
            return name < other.name;
        }
    };

    struct Arg
    {
        std::string name;
        std::string description;
        char nargs;

        bool operator<(const Arg &other) const
        {
            return name < other.name;
        }
    };

    static void init(const std::string &name, const std::string &description);
    static void addOpt(const Opt &option);
    static void addArg(const Arg &argument);

    static void parse(int argc, char **argv);

    static std::vector<std::string> getOpt(const std::string &name);
    static std::vector<std::string> getArg(const std::string &name);

    static std::string getHelp();
    static std::string getUsage();

private:

    static bool initialized;

    static std::string name;
    static std::string description;

    static std::map<std::string, std::string> shortToLong; // short name to long name
    static std::map<std::string, std::pair<Opt, std::vector<std::string>>> options;

    static std::vector<std::string> argNames;
    static std::map<std::string, std::pair<Arg, std::vector<std::string>>> arguments;
};

} // namespace mros