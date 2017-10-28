//
// Created by Peter Beno on 15/09/2017.
//

#include "FrameMatcher.h"
#include "Application.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <thread>





pcl::PointXYZRGB getCloudPoint(const pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y) {
    int arrayPosition = y*my_pcl.width + x * 1;

    pcl::PointXYZRGB cloudPoint = my_pcl.points[arrayPosition];

    return cloudPoint;
}


void colorizePoint(pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y, int r, int g, int b) {
    int arrayPosition =  y * my_pcl.width + x * 1;

    my_pcl.points[arrayPosition].r = r;
    my_pcl.points[arrayPosition].g = g;
    my_pcl.points[arrayPosition].b = b;

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



// using namespace std::literals;
void app::FrameMatcher::run() {

    // stuff for framematcher thread
    Frame previous_frame;

    while (app::Application::is_running) {

        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);
            // && !processed_frame.descriptors.empty() && !previous_frame.descriptors.empty()

            if (processed_frame.processed) {
//                std::cout << "matching!" << std::endl;




//                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB> ());
//                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB> ());
//
//                 // cv::Mat img_keypoints = processed_frame.claheMat.clone();//cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));//processed_frame.claheMat.clone();
//
//                 //-- Step 3: Matching descriptor vectors using Bruteforce matcher
//                 cv::BFMatcher matcher(cv::NORM_HAMMING);
//                 std::vector<cv::DMatch> matches;
//                 matcher.match(processed_frame.descriptors, previous_frame.descriptors, matches);
//
//
////                 // maps 3D point distances to matches, int is
////                 std::map<int, double> distances3DMap;
////                 std::vector<double> distances3D;
//
////                 double max_dist = 0;
////                 double min_dist = 1000000;
////                 //-- Quick calculation of max and min distances between keypoints
////                 for (int i = 0; i < matches.size(); i++) {
////                     double dist = matches[i].distance;
////                     if (dist < min_dist) min_dist = dist;
////                     if (dist > max_dist) max_dist = dist;
////                 }
////
////                 for (int i = 0; i < matches.size(); i++) {
////                     if (matches[i].distance < max_dist / 2) {
////                         // processed_frame.descriptors[matches[i].queryIdx][matches[i].trainIdx];
////                         auto currentFrameCVPoint = processed_frame.keypoints[matches[i].queryIdx].pt;
////                         auto previousFrameCVPoint = previous_frame.keypoints[matches[i].trainIdx].pt;
////
////                         auto currentCloudPoint = getCloudPoint(*(processed_frame).cloud, currentFrameCVPoint.x,
////                                                                currentFrameCVPoint.y);
////                         auto previousCloudPoint = getCloudPoint(*(previous_frame).cloud, previousFrameCVPoint.x,
////                                                                 previousFrameCVPoint.y);
////
////                         double distance3D = pcl::euclideanDistance<pcl::PointXYZRGB, pcl::PointXYZRGB>(
////                                 currentCloudPoint, previousCloudPoint);
////
////                         distances3DMap[i] = distance3D;
////                         distances3D.push_back(distance3D);
////                     }
////                 }
////
////                 double median3dDistanceMovement = calculateMedian(distances3D);
////                    std::cout << "median 3D distance movement: " << median3dDistanceMovement << std::endl;
//
//                 std::vector<cv::DMatch> good_matches;
//
//                 for (int i = 0; i < matches.size(); i++) {
////                     if (distances3DMap[i] != 0 && distances3DMap[i] < median3dDistanceMovement) {
//                         good_matches.push_back(matches[i]);
////                     }
//                 }
//
//                 // red is good
//                 for (int i = 0; i < good_matches.size(); i++) {
// //                        cv::circle(img_keypoints, processed_frame.keypoints[good_matches[i].queryIdx].pt, 1,
// //                                   cv::Scalar(0, 0, 255));
//                    colorizePoint(*processed_frame.cloud, processed_frame.keypoints[good_matches[i].queryIdx].pt.x,
//                                 processed_frame.keypoints[good_matches[i].queryIdx].pt.y, 255, 0 ,0);\
//
////
//// redundant
//                     auto currentFrameCVPoint = processed_frame.keypoints[good_matches[i].queryIdx].pt;
//                     auto previousFrameCVPoint = previous_frame.keypoints[good_matches[i].trainIdx].pt;
//                     pcl::PointXYZRGB  currentCloudPoint = getCloudPoint(*(processed_frame).cloud, currentFrameCVPoint.x,
//                                                                              currentFrameCVPoint.y);
//                     pcl::PointXYZRGB previousCloudPoint = getCloudPoint(*(previous_frame).cloud, previousFrameCVPoint.x,
//                                                             previousFrameCVPoint.y);
//                     cloud_in->push_back(currentCloudPoint);
//                     cloud_out->push_back(previousCloudPoint);
//
//                 }
//
//                 // SVD transformation estimation
//                 pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB>::Matrix4 transformation2;
//                 trasformation_estimation.estimateRigidTransformation (*cloud_in,*cloud_out,transformation2);
//                    std::cout << transformation2 << std::endl;
//
//






            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));


        previous_frame = processed_frame;
    }

    std::cout << "Frame Matcher exitting..." << std::endl;
}
