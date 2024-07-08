#include <iostream>
#include <sstream>
#include <vector>
#include <string>

#include "test_utils.hpp"

int main(int argc, char** argv)
{
    std::string line = "crossed_444_2.500.15_TIUS";

    std::vector<std::string> tokens = utils::splitString(line, '_');

    std::string structure_offset = tokens[2].substr(0, 4);

    std::cout << "Value: " << std::stod(structure_offset) << std::endl;
    return 0;
}