//
// Created by Peter Beno on 15/09/2017.
//

#ifndef APP_FRAMEVISUALIZER_H
#define APP_FRAMEVISUALIZER_H

#include "globals.h"


namespace app {
    class FrameMatcher {
    public:
        FrameMatcher(Options options,
        			 app::Frame &processed_frame, 
        			 std::mutex &processed_frame_mutex,
        			 app::Frame &matched_frame,
        			 std::mutex &matched_frame_mutex):
        		options(options),
        		processed_frame(processed_frame),
        		processed_frame_mutex(processed_frame_mutex),
                matched_frame(matched_frame), 
                matched_frame_mutex(matched_frame_mutex) {
        };

        Options options;
        Frame & processed_frame;
        std::mutex & processed_frame_mutex;
        Frame & matched_frame;
        std::mutex & matched_frame_mutex;

        void run();
    };
}

#endif //APP_FRAMEVISUALIZER_H
