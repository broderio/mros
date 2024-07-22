#include "mros/argparse.hpp"

int main(int argc, char **argv)
{
    ArgParser::init("test", "test description");
    ArgParser::addArg("arg1", "arg1 description");
    ArgParser::addArg("arg2", "arg2 description");
    ArgParser::addOption("option1", "o", "option1 description", "true");
    ArgParser::addOption("option2", "", "option2 description", "24");

    ArgParser::parse(argc, argv);

    std::cout << "arg1: " << ArgParser::getArg("arg1") << std::endl;
    std::cout << "arg2: " << ArgParser::getArg("arg2") << std::endl;
    std::cout << "option1: " << ArgParser::getOption("option1") << std::endl;
    std::cout << "option2: " <<  ArgParser::getOption("option2") << std::endl;

    std::cout << ArgParser::getHelp() << std::endl;

    return 0;
}