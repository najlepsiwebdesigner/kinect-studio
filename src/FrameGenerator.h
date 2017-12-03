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
        std::mutex & grabbed_frame_mutex;
        app::Frame & grabbed_frame;

        std::mutex & current_robot_pose_mutex;
        RobotPose & current_robot_pose;
        Options options;

        FrameGenerator(Options options,
                       app::Frame & grabbed_frame,
                       std::mutex & grabbed_frame_mutex,
                       RobotPose & _current_robot_pose,
                       std::mutex & _current_robot_pose_mutex):
                options(options),
                grabbed_frame(grabbed_frame),
                grabbed_frame_mutex(grabbed_frame_mutex),
                current_robot_pose(_current_robot_pose),
                current_robot_pose_mutex(_current_robot_pose_mutex){
        };

        void run();
    };
}

#endif //APP_FRAMEGENERATOR_H
