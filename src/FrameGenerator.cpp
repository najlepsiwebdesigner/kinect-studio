//
// Created by Peter Beno on 02/09/2017.
//

#include "FrameGenerator.h"
#include "offlinedatasource.h"
#include "Application.h"

namespace app {

    void FrameGenerator::run() {
        OfflineDataSource offlineDataSource;
        offlineDataSource.loop = true;

        cv::Mat video;
        cv::Mat depth;
        int i = 0;

        // while we have frames
        while (offlineDataSource.getVideoAndDepth(video, depth)) {

            if (! app::Application::is_running) break;

            // create frame
            Frame frame;

            frame.rgbMat = video;
            frame.depthMat = depth;

            frame.order = i;
            frame.unread = true;

            i++;

            // thread safe write to shared variable
            {
                std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);
                current_frame = frame;
            }

            // simulate some frame rate
            if (frequency)
                usleep(1000000 / frequency);
        }

        app::Application::stop();

        std::cout << "Generator thread exitting.." << std::endl;
    }

}