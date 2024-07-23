#include "mros/optparse.hpp"

namespace mros {

bool OptionParser::initialized = false;
std::string OptionParser::name;
std::string OptionParser::description;
std::map<std::string, std::string> OptionParser::shortToLong;
std::map<std::string, std::pair<OptionParser::Option, std::vector<std::string>>> OptionParser::options;

void OptionParser::init(const std::string &name, const std::string &description)
{
    if (OptionParser::initialized)
    {
        throw std::runtime_error("OptionParser already initialized");
    }

    OptionParser::name = name;
    OptionParser::description = description;
    OptionParser::initialized = true;
}

void OptionParser::addOption(const OptionParser::Option &option)
{
    if (!OptionParser::initialized)
    {
        throw std::runtime_error("OptionParser not initialized");
    }

    if (OptionParser::options.find(option.name) != OptionParser::options.end())
    {
        throw std::runtime_error("Option already exists");
    }

    if (option.nargs != '0' && option.nargs != '1' && option.nargs != '?' && option.nargs != '*' && option.nargs != '+')
    {
        throw std::runtime_error("Invalid nargs");
    }
    
    OptionParser::options[option.name] = {option, {option.defaultVal}};

    if (!option.shortName.empty())
    {
        OptionParser::shortToLong[option.shortName] = option.name;
    }
}

void OptionParser::parse(int argc, char **argv)
{
    if (!OptionParser::initialized)
    {
        throw std::runtime_error("OptionParser not initialized");
    }

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h")
        {
            throw std::runtime_error(getHelp());
        }

        // Check if it is an option
        if (arg[0] == '-')
        {
            std::string option;
            if (arg[1] == '-') // Long option
            {
                option = arg.substr(2);
            }
            else // Short option
            {
                if (arg.size() != 2 || OptionParser::shortToLong.find(arg.substr(1)) == OptionParser::shortToLong.end())
                {
                    throw std::runtime_error("Unknown option: " + arg + "\n" + getUsage());
                }

                option = OptionParser::shortToLong[arg.substr(1)];
            }

            if (OptionParser::options.find(option) == OptionParser::options.end())
            {
                throw std::runtime_error("Unknown option: " + arg + "\n" + getUsage());
            }

            OptionParser::Option opt = OptionParser::options[option].first;

            if (opt.nargs == '0')
            {
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    throw std::runtime_error("Option " + arg + " does not take arguments\n" + getUsage());
                }
                OptionParser::options[option].second[0] = opt.constVal;
            }
            else if (opt.nargs == '1')
            {
                if (i + 1 >= argc || argv[i + 1][0] == '-')
                {
                    throw std::runtime_error("Option " + arg + " requires an argument\n" + getUsage());
                }
                OptionParser::options[option].second[0] = argv[++i];
            }
            else if (opt.nargs == '?')
            {
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    OptionParser::options[option].second[0] = argv[++i];
                }
                else
                {
                    OptionParser::options[option].second[0] = opt.constVal;
                }
            }
            else if (opt.nargs == '*') // 0 or more
            {
                // Check if there is an argument
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    OptionParser::options[option].second.pop_back(); // Remove default value
                    while (i + 1 < argc && argv[i + 1][0] != '-')
                    {
                        OptionParser::options[option].second.push_back(argv[++i]);
                    }
                }
                else
                {
                    OptionParser::options[option].second[0] = opt.constVal;
                }
            }
            else if (opt.nargs == '+') // 1 or more
            {
                // Check if there is an argument
                if (i + 1 >= argc || argv[i + 1][0] == '-')
                {
                    throw std::runtime_error("Option " + arg + " requires at least one argument\n" + getUsage());
                }
                OptionParser::options[option].second.pop_back(); // Remove default value
                while (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    OptionParser::options[option].second.push_back(argv[++i]);
                }
            }
        }
        else 
        {
            throw std::runtime_error("Unknown argument: " + arg + "\n" + getUsage());
        }
    }

    // Check if required options are present
    for (const auto &option : OptionParser::options)
    {
        if (option.second.first.required && option.second.second[0] == option.second.first.defaultVal)
        {
            throw std::runtime_error("Required option " + option.first + " not present\n" + getUsage());
        }
    }
}

std::vector<std::string> OptionParser::getOption(const std::string &name)
{
    if (OptionParser::options.find(name) == OptionParser::options.end())
    {
        throw std::runtime_error("Unknown option: " + name + "\n" + getUsage());
    }

    return OptionParser::options[name].second;
}

std::string OptionParser::getHelp()
{
    if (!OptionParser::initialized)
    {
        throw std::runtime_error("OptionParser not initialized");
    }

    std::string help = "\n" + OptionParser::name + "\n" + OptionParser::description + "\n\n";

    help += OptionParser::getUsage() + "\n\n";

    help += "Options:\n";
    help += "  --help (-h): Show this help message\n\n";
    for (const auto &option : OptionParser::options)
    {
        help += "  --" + option.first + " ";
        if (!option.second.first.shortName.empty())
        {
            help += "(-" + option.second.first.shortName + "): ";
        }
        help += option.second.first.description + "\n";
        std::string reqStr = option.second.first.required ? "yes" : "no";
        help += ("    Required: " + reqStr + "\n");
        if (!option.second.first.required)
        {
            help += "    Default: " + option.second.first.defaultVal + "\n";
        }
        help += ("    Arguments: " + std::string(1, option.second.first.nargs) + "\n");
        if (option.second.first.nargs == '?' || option.second.first.nargs == '*')
        {
            help += "    Const value: " + option.second.first.constVal + "\n";
        }
        help += "\n";
    }

    return help;
}

std::string OptionParser::getUsage()
{
    if (!OptionParser::initialized)
    {
        throw std::runtime_error("OptionParser not initialized");
    }

    std::string usage = "Usage: " + OptionParser::name + " ";

    usage += "[--help (-h)] ";

    for (const auto &option : OptionParser::options)
    {
        if (!option.second.first.required)
        {
            usage += "[";
        }

        usage += "--" + option.first;
        if (!option.second.first.shortName.empty())
        {
            usage += " (-" + option.second.first.shortName + ")";
        }

        if (option.second.first.nargs == '1') // exactly 1
        {
            usage += " <value>";
        }
        else if (option.second.first.nargs == '?') // 0 or 1
        {
            usage += " [<value>]";
        }
        else if (option.second.first.nargs == '*') // 0 or more
        {
            usage += " [<value> ...]";
        }
        else if (option.second.first.nargs == '+') // 1 or more
        {
            usage += " <value> [<value> ...]";
        }

        if (!option.second.first.required)
        {
            usage += "] ";
        }
    }

    return usage;
}

} // namespace mros