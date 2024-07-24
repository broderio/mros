#include "mros/utils/argParser.hpp"

using namespace mros;

int main(int argc, char **argv)
{
    ArgParser::init("test", "test description");

    ArgParser::addOpt({"foo", "f", "foo description", "true", "false", '0'});
    ArgParser::addOpt({"jab", "j", "jab description", "forty", "", '1'});
    ArgParser::addOpt({"bar", "b", "bar description", "10", "20", '?'});

    ArgParser::addOpt({"zap", "z", "zap description", "one", "two", '*'});
    ArgParser::addOpt({"wok", "w", "wok description", "hello", "", '+'});

    ArgParser::addArg({"orb", "orb description", '1'});
    ArgParser::addArg({"lol", "lol description", '+'});

    ArgParser::parse(argc, argv);

    std::cout << "foo: " << ArgParser::getOpt("foo")[0] << std::endl;
    std::cout << "jab: " << ArgParser::getOpt("jab")[0] << std::endl;
    std::cout << "bar: " <<  ArgParser::getOpt("bar")[0] << std::endl;

    std::cout << "zap: ";
    for (const auto &option : ArgParser::getOpt("zap"))
    {
        std::cout << option << ", ";
    }
    std::cout << std::endl;

    std::cout << "wok: ";
    for (const auto &option : ArgParser::getOpt("wok"))
    {
        std::cout << option << ", ";
    }
    std::cout << std::endl;

    std::cout << "orb: " << ArgParser::getArg("orb")[0] << std::endl;

    std::cout << "lol: ";
    for (const auto &option : ArgParser::getArg("lol"))
    {
        std::cout << option << ", ";
    }
    std::cout << std::endl;

    std::cout << "\n\n" << ArgParser::getHelp() << std::endl;

    return 0;
}