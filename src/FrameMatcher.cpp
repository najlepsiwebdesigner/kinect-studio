//
// Created by Peter Beno on 15/09/2017.
//

#include "FrameMatcher.h"
#include "Application.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <thread>

void app::FrameMatcher::run() {
    using namespace std::literals;
//
//    cv::namedWindow("Depth");
//    cv::namedWindow("Video");
//    cv::namedWindow("Thresh");
//    cv::namedWindow("Good Matches");
//    cv::namedWindow("Keypoints");
//    cv::namedWindow("Clahe");
//
////    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
////    viewer->addCoordinateSystem (10.0);
////    viewer->initCameraParameters ();
//////
////    viewer->setBackgroundColor (0, 0, 0);
////    viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
////    viewer->setSize(640, 480);
//
    while (app::Application::is_running) {
//        Frame current_frame;
//        {
//            std::lock_guard<std::mutex> mutex_guard(visualized_frame_mutex);
//            current_frame = visualized_frame;
//
//
////            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(current_frame.cloud);
////            if (!viewer->updatePointCloud<pcl::PointXYZRGB>(current_frame.cloud, rgb, "sample cloud")) {
////                viewer->addPointCloud<pcl::PointXYZRGB>(current_frame.cloud, rgb, "sample cloud");
////            }
//
//            cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
//            current_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
//
//            cv::imshow("Video", current_frame.rgbMat);
////            cv::updateWindow("Video");
////            cv::imshow("Depth", depthf);
////            cv::updateWindow("Depth");
//
////            viewer->spinOnce(30);
//
//        }
//
        std::this_thread::sleep_for(100ms);
////
////        if (cvWaitKey(10)) {
////            app::Application::is_running = false;
////            break;
////        }

    }

    std::cout << "Frame Visualizer exitting..." << std::endl;
}
