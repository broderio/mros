#include "mros/argparse.hpp"

bool ArgParser::initialized = false;
std::string ArgParser::name;
std::string ArgParser::description;
std::map<std::string, ArgParser::Arg> ArgParser::args;
std::map<std::string, ArgParser::Option> ArgParser::options;

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

void ArgParser::addArg(const std::string &name, const std::string &description)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    if (ArgParser::args.find(name) != ArgParser::args.end())
    {
        throw std::runtime_error("Arg already exists");
    }

    ArgParser::args[name] = {description, ""};
}

void ArgParser::addOption(const std::string &name, const std::string &shortname, const std::string &description, const std::string &defaultValue)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    if (ArgParser::options.find(name) != ArgParser::options.end())
    {
        throw std::runtime_error("Option already exists");
    }

    ArgParser::options[name] = {shortname, description, defaultValue};
}

void ArgParser::parse(int argc, char **argv)
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    int requiredArgs = ArgParser::args.size();

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg[0] == '-')
        {
            if (arg[1] == '-')
            {
                std::string name = arg.substr(2);
                if (ArgParser::options.find(name) == ArgParser::options.end())
                {
                    throw std::runtime_error("Unknown option: " + name + "\nUsage: " + getUsage());
                }

                if (i + 1 < argc && argv[i + 1][0] != '-')
                {
                    ArgParser::options[name].value = argv[i + 1];
                    i++;
                }
            }
            else
            {
                std::string shortname = arg.substr(1);
                bool found = false;
                for (auto &option : ArgParser::options)
                {
                    if (option.second.shortname == shortname)
                    {
                        found = true;
                        if (i + 1 < argc && argv[i + 1][0] != '-')
                        {
                            option.second.value = argv[i + 1];
                            i++;
                        }
                        break;
                    }
                }

                if (!found)
                {
                    throw std::runtime_error("Unknown option: " + shortname + "\nUsage: " + getUsage());
                }
            }
        }
        else
        {
            if (ArgParser::args.size() == 0)
            {
                throw std::runtime_error("Unknown argument: " + arg + "\nUsage: " + getUsage());
            }

            bool found = false;
            for (auto &_arg : ArgParser::args)
            {
                if (_arg.second.value == "")
                {
                    _arg.second.value = arg;
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error("Too many arguments\nUsage: " + getUsage());
            }

            requiredArgs--;
        }
    }

    if (requiredArgs > 0)
    {
        throw std::runtime_error("Missing arguments\nUsage: " + getUsage());
    }
}

std::string ArgParser::getArg(const std::string &name)
{
    if (ArgParser::args.find(name) == ArgParser::args.end())
    {
        throw std::runtime_error("Unknown argument: " + name + "\nUsage: " + getUsage());
    }

    return ArgParser::args[name].value;
}

std::string ArgParser::getOption(const std::string &name)
{
    if (ArgParser::options.find(name) == ArgParser::options.end())
    {
        throw std::runtime_error("Unknown option: " + name + "\nUsage: " + getUsage());
    }

    return ArgParser::options[name].value;
}

bool ArgParser::hasOption(const std::string &name)
{
    if (ArgParser::options.find(name) == ArgParser::options.end())
    {
        throw std::runtime_error("Unknown option: " + name + "\nUsage: " + getUsage());
    }

    return ArgParser::options[name].value != "";
}

std::string ArgParser::getHelp()
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    std::string help = ArgParser::name + " - " + ArgParser::description + "\n\n";
    help += "Arguments:\n";
    for (auto &arg : ArgParser::args)
    {
        help += "  " + arg.first + " - " + arg.second.description + "\n";
    }

    help += "\nOptions:\n";
    for (auto &option : ArgParser::options)
    {
        help += "  --" + option.first;
        if (option.second.shortname != "")
        {
            help += " -" + option.second.shortname;
        }
        help += " - " + option.second.description + "\n";
    }

    return help;
}

std::string ArgParser::getUsage()
{
    if (!ArgParser::initialized)
    {
        throw std::runtime_error("ArgParser not initialized");
    }

    std::string usage = ArgParser::name + " ";
    for (auto &arg : ArgParser::args)
    {
        usage += "<" + arg.first + "> ";
    }

    for (auto &option : ArgParser::options)
    {
        usage += "[--" + option.first;
        if (option.second.shortname != "")
        {
            usage += " -" + option.second.shortname;
        }
        usage +=  "] ";
    }

    return usage;
}