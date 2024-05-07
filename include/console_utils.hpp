#pragma once

#include <iostream>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define ORANGE  "\033[35m"

using namespace std;

namespace utils{

class Console
{
public:
    Console(/* args */);
    ~Console();

    void info(const std::string msg, const std::string color = BLUE);
    void debug(const std::string msg, const std::string color = YELLOW);

    bool enable;
    bool enable_vis;
};

Console::Console(/* args */)
{
    enable = true;
    enable_vis = true;
}

Console::~Console()
{
    enable = false;
    enable_vis = false;
}


void Console::info(const string msg, const string color)
{
    std::cout << color << msg << RESET << std::endl;
}

void Console::debug(const string msg, const string color)
{
    if (this->enable){
        std::cout << color << msg << RESET << std::endl;
    }
}

} // namespace arvc