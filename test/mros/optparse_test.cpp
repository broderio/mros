#include "mros/optparse.hpp"

using namespace mros;

int main(int argc, char **argv)
{
    OptionParser::init("test", "test description");
    OptionParser::addOption({"foo", "f", "foo description", "true", "false", '0'});
    OptionParser::addOption({"bar", "b", "bar description", "10", "10", '?'});
    OptionParser::addOption({"zap", "z", "zap description", "one", "one", '*'});

    OptionParser::parse(argc, argv);

    std::cout << "foo: " << OptionParser::getOption("foo")[0] << std::endl;
    std::cout << "bar: " <<  OptionParser::getOption("bar")[0] << std::endl;

    std::cout << "zap: ";
    for (const auto &option : OptionParser::getOption("zap"))
    {
        std::cout << option << ", ";
    }


    std::cout << "\n\n" << OptionParser::getHelp() << std::endl;

    return 0;
}