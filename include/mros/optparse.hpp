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

class OptionParser
{
public:
    struct Option
    {
        std::string name;
        std::string shortName;
        std::string description;
        std::string defaultVal; // value if not specified
        std::string constVal; // value if specified without argument
        char nargs;
        bool required;

        bool operator<(const Option &other) const
        {
            return name < other.name;
        }
    };

    static void init(const std::string &name, const std::string &description);
    static void addOption(const Option &option);

    static void parse(int argc, char **argv);

    static std::vector<std::string> getOption(const std::string &name);

    static std::string getHelp();
    static std::string getUsage();

private:

    static bool initialized;

    static std::string name;
    static std::string description;

    static std::map<std::string, std::string> shortToLong; // short name to long name
    static std::map<std::string, std::pair<Option, std::vector<std::string>>> options;
};

} // namespace mros