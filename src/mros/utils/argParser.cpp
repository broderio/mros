#include "mros/utils/argParser.hpp"

namespace mros {

bool ArgParser::initialized = false;
std::string ArgParser::name;
std::string ArgParser::description;
std::map<std::string, std::string> ArgParser::shortToLong;
std::map<std::string, std::pair<ArgParser::Opt, std::vector<std::string>>> ArgParser::options;
std::vector<std::string> ArgParser::argNames;
std::map<std::string, std::pair<ArgParser::Arg, std::vector<std::string>>> ArgParser::arguments;

void ArgParser::init(const std::string &name, const std::string &description)
{
    if (ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser already initialized");
    }

    ArgParser::name = name;
    ArgParser::description = description;
    ArgParser::initialized = true;
}

void ArgParser::addOpt(const ArgParser::Opt &option)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    if (ArgParser::options.find(option.name) != ArgParser::options.end())
    {
        throw std::runtime_error("Opt already exists");
    }

    if (option.nargs != '0' && option.nargs != '1' && option.nargs != '?' && option.nargs != '*' && option.nargs != '+')
    {
        throw std::runtime_error("Invalid nargs");
    }
    
    ArgParser::options[option.name] = {option, {option.defaultVal}};

    if (!option.shortName.empty())
    {
        ArgParser::shortToLong[option.shortName] = option.name;
    }
}

void ArgParser::addArg(const ArgParser::Arg &argument)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    if (ArgParser::arguments.find(argument.name) != ArgParser::arguments.end())
    {
        throw std::runtime_error("Arg already exists");
    }

    if (argument.nargs != '1' && argument.nargs != '+')
    {
        throw std::runtime_error("Invalid nargs");
    }

    ArgParser::argNames.push_back(argument.name);
    ArgParser::arguments[argument.name] = {argument, {}};
}

void ArgParser::parse(int argc, char **argv)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    int j = 0;
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
                if (arg.size() != 2 || ArgParser::shortToLong.find(arg.substr(1)) == ArgParser::shortToLong.end())
                {
                    throw std::runtime_error("Unknown option: " + arg + "\n" + getUsage());
                }

                option = ArgParser::shortToLong[arg.substr(1)];
            }

            if (ArgParser::options.find(option) == ArgParser::options.end())
            {
                throw std::runtime_error("Unknown option: " + arg + "\n" + getUsage());
            }

            ArgParser::Opt opt = ArgParser::options[option].first;

            if (opt.nargs == '0') // exactly 0
            {
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    throw std::runtime_error("Opt " + arg + " does not take arguments\n" + getUsage());
                }
                ArgParser::options[option].second[0] = opt.constVal;
            }
            else if (opt.nargs == '1') // exactly 1
            {
                if (i + 1 >= argc || argv[i + 1][0] == '-')
                {
                    throw std::runtime_error("Opt " + arg + " requires an argument\n" + getUsage());
                }
                ArgParser::options[option].second[0] = argv[++i];
            }
            else if (opt.nargs == '?') // 0 or 1
            {
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    ArgParser::options[option].second[0] = argv[++i];
                }
                else
                {
                    ArgParser::options[option].second[0] = opt.constVal;
                }
            }
            else if (opt.nargs == '*') // 0 or more
            {
                // Check if there is an argument
                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    ArgParser::options[option].second.pop_back(); // Remove default value
                    while (i + 1 < argc && argv[i + 1][0] != '-')
                    {
                        ArgParser::options[option].second.push_back(argv[++i]);
                    }
                }
                else
                {
                    ArgParser::options[option].second[0] = opt.constVal;
                }
            }
            else if (opt.nargs == '+') // 1 or more
            {
                // Check if there is an argument
                if (i + 1 >= argc || argv[i + 1][0] == '-')
                {
                    throw std::runtime_error("Opt " + arg + " requires at least one argument\n" + getUsage());
                }
                ArgParser::options[option].second.pop_back(); // Remove default value
                while (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    ArgParser::options[option].second.push_back(argv[++i]);
                }
            }
        }
        else 
        {
            // Check if it is an argument
            if (j >= ArgParser::argNames.size())
            {
                throw std::runtime_error("Unknown argument: " + arg + "\n" + getUsage());
            }

            ArgParser::Arg argument = ArgParser::arguments[ArgParser::argNames[j]].first;

            if (argument.nargs == '1')
            {
                ArgParser::arguments[ArgParser::argNames[j]].second.push_back(arg);
            }
            else if (argument.nargs == '+') // 1 or more
            {
                while (i < argc && argv[i][0] != '-')
                {
                    ArgParser::arguments[ArgParser::argNames[j]].second.push_back(argv[i++]);
                }
                --i; // To account for the extra increment in the for loop
            }

            j++;
        }
    }

    // Check if all required arguments are present
    for (const auto &argument : ArgParser::arguments)
    {
        if (argument.second.first.nargs == '1' && argument.second.second.empty())
        {
            throw std::runtime_error("Argument " + argument.second.first.name + " requires an argument\n" + getUsage());
        }
        else if (argument.second.first.nargs == '+' && argument.second.second.empty())
        {
            throw std::runtime_error("Argument " + argument.second.first.name + " requires at least one argument\n" + getUsage());
        }
    }
}

std::vector<std::string> ArgParser::getOpt(const std::string &name)
{
    if (ArgParser::options.find(name) == ArgParser::options.end())
    {
        throw std::runtime_error("Unknown option: " + name + "\n" + getUsage());
    }

    return ArgParser::options[name].second;
}

std::vector<std::string> ArgParser::getArg(const std::string &name)
{
    if (ArgParser::arguments.find(name) == ArgParser::arguments.end())
    {
        throw std::runtime_error("Unknown argument: " + name + "\n" + getUsage());
    }

    return ArgParser::arguments[name].second;
}

std::string ArgParser::getHelp()
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    std::string help = "\n" + ArgParser::name + "\n" + ArgParser::description + "\n\n";

    help += "PROG [--help, -h] ARGS [OPTIONS]\n\n";
    help += ArgParser::getUsage() + "\n\n";

    help += "Args:\n";
    for (const auto &argument : ArgParser::arguments)
    {
        help += "  " + argument.second.first.name + ": " + argument.second.first.description + "\n";
        help += "    Arguments: " + std::string(1, argument.second.first.nargs) + "\n\n";
    }

    help += "Opts:\n";
    help += "  --help (-h): Show this help message\n\n";
    for (const auto &option : ArgParser::options)
    {
        help += "  --" + option.first + " ";
        if (!option.second.first.shortName.empty())
        {
            help += "(-" + option.second.first.shortName + "): ";
        }
        help += option.second.first.description + "\n";
        help += "    Default: " + option.second.first.defaultVal + "\n";
        help += ("    Arguments: " + std::string(1, option.second.first.nargs) + "\n");
        if (option.second.first.nargs == '?' || option.second.first.nargs == '*')
        {
            help += "    Const value: " + option.second.first.constVal + "\n";
        }
        help += "\n";
    }

    return help;
}

std::string ArgParser::getUsage()
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    std::string usage = "Usage: " + ArgParser::name + " ";
    usage += "[--help (-h)] ";

    for (const std::string &argName : ArgParser::argNames)
    {
        ArgParser::Arg argument = ArgParser::arguments[argName].first;

        if (argument.nargs == '1') // exactly 1
        {
            usage += "<" + argument.name + ">";
        }
        else if (argument.nargs == '+') // 1 or more
        {
            usage += "<" + argument.name + ">" + " [<" + argument.name + "> ...]";
        }

        usage += " ";
    }
    
    for (const auto &option : ArgParser::options)
    {
        usage += "[--" + option.first;
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

        usage += "] ";
    }

    return usage;
}

} // namespace mros