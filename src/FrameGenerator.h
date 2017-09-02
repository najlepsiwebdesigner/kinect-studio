//
// Created by Peter Beno on 02/09/2017.
//



#ifndef APP_FRAMEGENERATOR_H
#define APP_FRAMEGENERATOR_H

#include "globals.h"

namespace app {

    class FrameGenerator {
    public:
        int frequency = 0; // 0 = infinite
        std::mutex & current_frame_mutex;
        app::Frame & current_frame;

        FrameGenerator(app::Frame & _current_frame, std::mutex & _current_frame_mutex):
                current_frame(_current_frame),
                current_frame_mutex(_current_frame_mutex) {
        };

        void run();
    };
}

#endif //APP_FRAMEGENERATOR_H
