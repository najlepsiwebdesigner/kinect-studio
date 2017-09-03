//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"






bool app::Application::is_running = false;

void app::Application::start() {




    cv::namedWindow("Depth");
    cv::namedWindow("Video");
    cv::namedWindow("Thresh");
    cv::namedWindow("Good Matches");
    cv::namedWindow("Keypoints");
    cv::namedWindow("Clahe");


    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(grabbed_frame, grabbed_frame_mutex);
    FrameProcessor frameProcessor(grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);

    Frame previous_frame;

    is_running = true;



    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });



    bool is_first = true;


    while (is_running) {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);





            if (processed_frame.processed) {

                cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

//                cv::Mat threshf(cv::Size(640, 480), CV_8UC1);
//                processed_frame.thresholdedDepthMat.convertTo(threshf, CV_8UC1, 255.0 / 2048.0);

//                cv::Mat baf(cv::Size(640, 480), CV_8UC1);
//                processed_frame.baMat.convertTo(baf, CV_8UC1, 1);

                cv::imshow("Video", processed_frame.rgbMat);
                cv::imshow("Depth", depthf);
//                cv::imshow("Clahe", processed_frame.claheMat);








                cv::Mat img_keypoints = processed_frame.claheMat.clone();//cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));//processed_frame.claheMat.clone();

                // blue is previous
                for (const auto & keypoint: previous_frame.keypoints) {
                    cv::circle(img_keypoints, keypoint.pt,1,cv::Scalar(255,0,0));

                }

                // green is current
                for (const auto & keypoint: processed_frame.keypoints) {
                    cv::circle(img_keypoints, keypoint.pt,1,cv::Scalar(0,255,0));

                }




                if (!processed_frame.descriptors.empty() && !previous_frame.descriptors.empty()) {

//                    usleep(1000*1000);

                    //-- Step 3: Matching descriptor vectors using Bruteforce matcher
                    cv::BFMatcher matcher;
                    std::vector<cv::DMatch> matches;
                    matcher.match(processed_frame.descriptors, previous_frame.descriptors, matches);

                    double max_dist = 0;
                    double min_dist = 1000000;

                    //-- Quick calculation of max and min distances between keypoints
                    for (int i = 0; i < processed_frame.descriptors.rows; i++) {
                        double dist = matches[i].distance;
                        if (dist < min_dist) min_dist = dist;
                        if (dist > max_dist) max_dist = dist;
                    }

//                    printf("-- Max dist : %f \n", max_dist);
//                    printf("-- Min dist : %f \n", min_dist);


                    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
                    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
                    //-- small)
                    //-- PS.- radiusMatch can also be used here.
                    std::vector<cv::DMatch> good_matches;

                    for (int i = 0; i < processed_frame.descriptors.rows; i++) {
                        if (matches[i].distance < 200) { good_matches.push_back(matches[i]); }
                    }

                    // red is good
                    for (const auto & match: good_matches) {
                        cv::circle(img_keypoints, processed_frame.keypoints[match.queryIdx].pt,1,cv::Scalar(0,0,255));

                    }


                    //                cv::Mat sobelf = baf.clone();
                    //                sobel(sobelf);
                    //                sobel(sobelf);
                }

                cv::imshow("Keypoints", img_keypoints);










//                cv::imshow("BA",baf);

//                if (is_first) {
//                    is_first = false;
//                } else {
//                    matchAndDraw(previous_processed_baf,baf);
//                }
//


                previous_frame = processed_frame;
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