//
// Created by Peter Beno on 02/09/2017.
//

#include "FrameGenerator.h"
#include "offlinedatasource.h"
#include "kinectdatasource.h"
#include "Application.h"

// using namespace std::chrono_literals;

namespace app {
    // using namespace std::chrono_literals;
    
    void FrameGenerator::run() {
        KinectDataSource offlineDataSource;
//        offlineDataSource.loop = true;

        cv::Mat video;
        cv::Mat depth;
        int i = 0;

        // while we have frames
        while (offlineDataSource.getVideoAndDepth(video, depth)) {

            if (! app::Application::is_running) break;


            if (depth.cols == 0) {
                continue;
            }

            // create frame
            Frame frame;

            frame.rgbMat = video.clone();
            frame.depthMat = depth.clone();

            frame.order = i;
            frame.unread = true;

            i++;

            // thread safe write to shared variable
            {
                std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);
                current_frame = frame;
            }

            // simulate some frame rate
//            if (frequency)
                // usleep(1000000 / frequency);
//                std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        app::Application::stop();

        std::cout << "Generator thread exitting.." << std::endl;
    }

}