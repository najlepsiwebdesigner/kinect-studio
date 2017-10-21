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

        std::mutex & current_robot_pose_mutex;
        RobotPose & current_robot_pose;

        FrameGenerator(app::Frame & _current_frame,
                       std::mutex & _current_frame_mutex,
                       RobotPose & _current_robot_pose,
                       std::mutex & _current_robot_pose_mutex):
                current_frame(_current_frame),
                current_frame_mutex(_current_frame_mutex),
                current_robot_pose(_current_robot_pose),
                current_robot_pose_mutex(_current_robot_pose_mutex){
        };

        void run();
    };
}

#endif //APP_FRAMEGENERATOR_H
