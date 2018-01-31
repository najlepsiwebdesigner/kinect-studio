#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

class Logger {
public:
    template <typename T> static  void log(T input) {
        std::cout << "line" << std::endl;
    }



};

#endif