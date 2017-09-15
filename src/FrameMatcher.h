//
// Created by Peter Beno on 15/09/2017.
//

#ifndef APP_FRAMEVISUALIZER_H
#define APP_FRAMEVISUALIZER_H

#include "globals.h"


namespace app {
    class FrameMatcher {
    public:
        FrameMatcher(app::Frame &_visualized_frame, std::mutex &_visualized_frame_mutex):
                visualized_frame(_visualized_frame), visualized_frame_mutex(_visualized_frame_mutex) {
        };


        Frame & visualized_frame;
        std::mutex & visualized_frame_mutex;

        void run();
    };
}

#endif //APP_FRAMEVISUALIZER_H
