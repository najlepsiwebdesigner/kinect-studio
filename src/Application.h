//
// Created by Peter Beno on 02/09/2017.
//


#ifndef APP_APPLICATION_H
#define APP_APPLICATION_H

#include "globals.h"
#include "Frame.h"
#include "FrameGenerator.h"
#include "FrameProcessor.h"

namespace app {
    class Application {

    public:
        static bool is_running;

        static void start();

        static void stop();

    };
}

#endif //APP_APPLICATION_H
