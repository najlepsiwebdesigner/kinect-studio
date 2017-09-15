//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"

#include <map>
#include <pcl/common/distances.h>
#include <pcl/visualization/cloud_viewer.h>






pcl::PointXYZRGB getCloudPoint(const pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y) {
    int arrayPosition = y*my_pcl.width + x * 1;

    pcl::PointXYZRGB cloudPoint = my_pcl.points[arrayPosition];

    return cloudPoint;
}


void makePointRed(pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y) {
    int arrayPosition =  y * my_pcl.width + x * 1;

    my_pcl.points[arrayPosition].r = 255;
    my_pcl.points[arrayPosition].g = 0;
    my_pcl.points[arrayPosition].b = 0;

}



double calculateMedian(std::vector<double> scores)
{
  if (scores.size() < 1) {
      return 0;
  }
  double median;
  size_t size = scores.size();

  std::sort(scores.begin(), scores.end());

  if (size  % 2 == 0)
  {
      median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
  }
  else
  {
      median = scores[size / 2];
  }

  return median;
}





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



    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (10.0);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
    viewer->setSize(640, 480);


    while (is_running) {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);


            if (processed_frame.processed) {
                cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

                cv::imshow("Video", processed_frame.rgbMat);
                cv::imshow("Depth", depthf);


                cv::Mat img_keypoints = processed_frame.claheMat.clone();//cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));//processed_frame.claheMat.clone();
//
//
////                std::cout << "Number of keypoints: " << processed_frame.keypoints.size() << std::endl;
////
////                // blue are all keypoints in previous
////                for (const auto & keypoint: previous_frame.keypoints) {
////                    cv::circle(img_keypoints, keypoint.pt,1,cv::Scalar(255,0,0));
////
////                }
////
////                // green are depth keypoints in previous
////                for (const auto & keypoint: processed_frame.keypoints) {
////                    cv::circle(img_keypoints, keypoint.pt,1,cv::Scalar(0,255,0));
////
////                }
//
////
////                // green is current
////                for (const auto & keypoint: processed_frame.keypoints) {
////                    if (processed_frame.depthMat.at<uint16_t>(keypoint.pt) != 0) {
////                        cv::circle(img_keypoints, keypoint.pt, 1, cv::Scalar(0, 255, 0));
////                    } else {
////                        auto i = &keypoint - &processed_frame.keypoints[0];
////
////                    }
////                }
//
//
//
////                // green is current
////                for (const auto & keypoint: processed_frame.keypoints) {
////
////                    if (processed_frame.depthMat.at<uint16_t>(keypoint.pt) != 0) {
////                        cv::circle(img_keypoints, keypoint.pt, 1, cv::Scalar(0, 255, 0));
////                    }
////                }
//
////
////
//
                if (!processed_frame.descriptors.empty() && !previous_frame.descriptors.empty()) {
                    //-- Step 3: Matching descriptor vectors using Bruteforce matcher
                    cv::BFMatcher matcher(cv::NORM_HAMMING);
                    std::vector<cv::DMatch> matches;
                    matcher.match(processed_frame.descriptors, previous_frame.descriptors, matches);


                    // maps 3D point distances to matches, int is
                    std::map<int, double> distances3DMap;
                    std::vector<double> distances3D;

                    for (int i = 0; i < matches.size(); i++) {
                        if (matches[i].distance < 500) {
                            // processed_frame.descriptors[matches[i].queryIdx][matches[i].trainIdx];
                            auto currentFrameCVPoint = processed_frame.keypoints[matches[i].queryIdx].pt;
                            auto previousFrameCVPoint = previous_frame.keypoints[matches[i].trainIdx].pt;

                            auto currentCloudPoint = getCloudPoint(*(processed_frame).cloud, currentFrameCVPoint.x,
                                                                   currentFrameCVPoint.y);
                            auto previousCloudPoint = getCloudPoint(*(processed_frame).cloud, previousFrameCVPoint.x,
                                                                    previousFrameCVPoint.y);

                            double distance3D = pcl::euclideanDistance<pcl::PointXYZRGB, pcl::PointXYZRGB>(
                                    currentCloudPoint, previousCloudPoint);

                            distances3DMap[i] = distance3D;
                            distances3D.push_back(distance3D);
                        }
                    }

                    double medianDistanceMovement = calculateMedian(distances3D);
                    std::cout << "median 3D distance movement: " << medianDistanceMovement << std::endl;

                    std::vector<cv::DMatch> good_matches;

                    for (int i = 0; i < matches.size(); i++) {
                        if (distances3DMap[i] != 0 && distances3DMap[i] < medianDistanceMovement){
                            good_matches.push_back(matches[i]);
                        }
                    }

                    // red is good
                    for (const auto & match: good_matches) {
                        cv::circle(img_keypoints, processed_frame.keypoints[match.queryIdx].pt,1,cv::Scalar(0,0,255));
                        makePointRed(*processed_frame.cloud, processed_frame.keypoints[match.queryIdx].pt.x, processed_frame.keypoints[match.queryIdx].pt.y);


//                        cv::circle(img_keypoints, cv::Point(200,100),1,cv::Scalar(0,0,255));
//                        makePointRed(*processed_frame.cloud, 200, 100);


                    }

                    std::cout << "number of good matches: " << good_matches.size() << std::endl;

                }








                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(processed_frame.cloud);
                if (!viewer->updatePointCloud<pcl::PointXYZRGB> (processed_frame.cloud,rgb, "sample cloud")) {
                    viewer->addPointCloud<pcl::PointXYZRGB> (processed_frame.cloud, rgb, "sample cloud");
//                    viewer->resetCameraViewpoint("sample cloud");
                }
                viewer->spinOnce (1);





//
//
//
//
//
//
//
//
//////                    double max_dist = 0;
//////                    double min_dist = 1000000;
//////
//////                    //-- Quick calculation of max and min distances between keypoints
//////                    for (int i = 0; i < processed_frame.descriptors.rows; i++) {
//////                        double dist = matches[i].distance;
//////                        if (dist < min_dist) min_dist = dist;
//////                        if (dist > max_dist) max_dist = dist;
//////                    }
////
//////                    printf("-- Max dist : %f \n", max_dist);
//////                    printf("-- Min dist : %f \n", min_dist);
////
////
//
////
////                    double medianDistanceMovement = calculateMedian(distances);
////                    std::cout << "median 3D distance movement: " << medianDistanceMovement << std::endl;
////
////
////                    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
////                    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
////                    //-- small)
////                    //-- PS.- radiusMatch can also be used here.
////                    std::vector<cv::DMatch> good_matches;
////
////                    for (int i = 0; i < processed_frame.descriptors.rows; i++) {
//////
//////
//////
//////                        if (matches[i].distance < 250) {
//////
//////
////////                            processed_frame.descriptors[matches[i].queryIdx][matches[i].trainIdx];
//////                            auto currentFrameCVPoint = processed_frame.keypoints[matches[i].queryIdx].pt;
//////                            auto previousFrameCVPoint = previous_frame.keypoints[matches[i].trainIdx].pt;
//////
//////                            auto currentCloudPoint = getCloudPoint(*(processed_frame).cloud, currentFrameCVPoint.x, currentFrameCVPoint.y);
//////                            auto previousCloudPoint = getCloudPoint(*(processed_frame).cloud, previousFrameCVPoint.x, previousFrameCVPoint.y);
//////
//////
////////                            if (processed_frame.depthMat.at<float>(currentFrameCVPoint.x, currentFrameCVPoint.y) == 0) {
////////                                std::cout << "skipped point" << std::endl;
////////                                continue;
////////                            }
//////
//////
////////                            std::cout << currentCloudPoint << std::endl;
////////                            std::cout << previousCloudPoint << std::endl;
//////
//////
//////                            double distance3D = pcl::euclideanDistance<pcl::PointXYZRGB,pcl::PointXYZRGB>(currentCloudPoint, previousCloudPoint);
//////
//////
//////
////////                            if (processed_frame.depthMat.at<float>(currentFrameCVPoint.x, currentFrameCVPoint.y) != 0 ) {
//////
//////                                if (distance3D < 5) {
////                                    good_matches.push_back(matches[i]);
//////                                }
//////
////////                            }
//////                        }
////                    }
////
////
////                    std::cout <<  good_matches.size() << std::endl;
////
////
//
//
////                }
//
                cv::imshow("Keypoints", img_keypoints);
//
//
//
//
//
//
//
//
//
//
////                cv::imshow("BA",processed_frame.baMat);
//
////                if (is_first) {
////                    is_first = false;
////                } else {
////                    matchAndDraw(previous_processed_baf,baf);
////                }
////
//
//
                previous_frame = processed_frame;
            }

////            std::lock_guard<std::mutex> mutex_guard(grabbed_frame_mutex);
////
////            cv::imshow("Depth", grabbed_frame.depthMat);
////            cv::imshow("Video", grabbed_frame.rgbMat);

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