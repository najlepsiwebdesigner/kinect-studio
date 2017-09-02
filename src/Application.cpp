//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"

bool app::Application::is_running = false;

void app::Application::start() {
    cv::namedWindow("Depth");
    cv::namedWindow("Video");
    cv::namedWindow("Thresh");

    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(grabbed_frame, grabbed_frame_mutex);
    FrameProcessor frameProcessor(grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);

    is_running = true;



    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });




    while (is_running) {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);

            if (processed_frame.processed) {
                cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

                cv::Mat threshf(cv::Size(640, 480), CV_8UC1);
                processed_frame.thresholdedDepthMat.convertTo(threshf, CV_8UC1, 255.0 / 2048.0);

                cv::imshow("Video", processed_frame.rgbMat);
                cv::imshow("Depth", depthf);
                cv::imshow("Thresh", threshf);
            }


//            std::lock_guard<std::mutex> mutex_guard(grabbed_frame_mutex);
//
//            cv::imshow("Depth", grabbed_frame.depthMat);
//            cv::imshow("Video", grabbed_frame.rgbMat);
        }

        if (cvWaitKey(10) >= 0) {
            is_running = false;
            break;
        }
    }


    t1.join();
    t2.join();

}

void app::Application::stop() {
    is_running = false;
}