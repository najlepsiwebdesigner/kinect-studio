//
// Created by Peter Beno on 15/09/2017.
//

#include "FrameMatcher.h"
#include "Application.h"

#include <chrono>
#include <thread>

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;
using namespace app;


pcl::PointXYZRGB & getCloudPoint(pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y) {
    int arrayPosition = y*my_pcl.width + x * 1;

    pcl::PointXYZRGB & cloudPoint = my_pcl.points[arrayPosition];

    return cloudPoint;
}


Eigen::Affine3f estimateVisualTransformation(app::Frame & frame1, app::Frame & frame2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud) {

    // static long long iii = 0;


    // cv::Mat img1 = frame1.rgbMat;
    // cv::Mat img2 = frame2.rgbMat;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (frame1.cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (frame2.cloud);


    const int MAXIMAL_FEATURE_DISTANCE = 20;
    /// ### feature detection ± 13 ms
    // Ptr<SURF> detector = SURF::create( 100,4,1,false,false );
    // Ptr<AKAZE> detector = AKAZE::create();
    // Ptr<ORB> detector = ORB::create(1000);
    //         Ptr<SIFT> detector = SIFT::create();
    // Ptr<MSER> detector = MSER::create();
            // Ptr<BRISK> detector = BRISK::create();
            // Ptr<KAZE> detector = KAZE::create();
    // Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();

    std::vector<KeyPoint> & keypoints1 = frame1.keypoints;
    std::vector<KeyPoint> & keypoints2 = frame2.keypoints;
    // detector->detect( img1, keypoints1);
    // detector->detect( img2, keypoints2);

    /// ### feature description
    // Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
    // Ptr<SURF> extractor = SURF::create();
    Mat & descriptors1 = frame1.descriptors;
    Mat & descriptors2 = frame2.descriptors;

    // extractor->compute(img1, keypoints1, descriptors1);
    // extractor->compute(img2, keypoints2, descriptors2);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // std::cout << "number of matches: " << matches.size() << std::endl;

    std::vector<Vector3f> movement_vectors;
    Vector3f average_movement_vector;
    int number_of_matches = 0;

    std::vector< DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++ ) {
        // std::cout << "d: " << matches[i].distance << std::endl;

        if( matches[i].distance < MAXIMAL_FEATURE_DISTANCE) {
            good_matches.push_back( matches[i]);
        }
    }

    // std::cout << "number of good matches: " << matches.size() << std::endl;


    // create feature point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);


    // long double average_z_movement = 0;
    // int z_movement_counter = 0;

    // for (int i = 0; i < good_matches.size(); i++) {
    //   auto cvPoint1 = keypoints1[matches[i].queryIdx].pt;
    //   auto cvPoint2 = keypoints2[matches[i].trainIdx].pt;

    //   auto & cloudpoint1 = getCloudPoint(*cloud1, cvPoint1.x,cvPoint1.y);
    //   auto & cloudpoint2 = getCloudPoint(*cloud2, cvPoint2.x,cvPoint2.y);

    //   if (cloudpoint1.x == 0 && cloudpoint1.y == 0 && cloudpoint1.z == 0) continue;
    //   if (cloudpoint2.x == 0 && cloudpoint2.y == 0 && cloudpoint2.z == 0) continue;
    //   if (fabs(cloudpoint1.y - cloudpoint2.y) > 200) continue;

    //   average_z_movement += fabs(cloudpoint1.z - cloudpoint2.z);
    //   z_movement_counter++;
    // }

    // average_z_movement = average_z_movement/z_movement_counter;

    for (int i = 0; i < good_matches.size(); i++) {
        auto cvPoint1 = keypoints1[matches[i].queryIdx].pt;
        auto cvPoint2 = keypoints2[matches[i].trainIdx].pt;

        auto & cloudpoint1 = getCloudPoint(*cloud1, cvPoint1.x,cvPoint1.y);
        auto & cloudpoint2 = getCloudPoint(*cloud2, cvPoint2.x,cvPoint2.y);

        if (cloudpoint1.x == 0 && cloudpoint1.y == 0 && cloudpoint1.z == 0) continue;
        if (cloudpoint2.x == 0 && cloudpoint2.y == 0 && cloudpoint2.z == 0) continue;
        if (fabs(cloudpoint1.y - cloudpoint2.y) > 100) continue;
        // if (fabs(cloudpoint1.z - cloudpoint2.z) > (average_z_movement * 1.5) || fabs(cloudpoint1.z - cloudpoint2.z) < (average_z_movement * 0.5)) continue;

        // viewer->addSphere(cloudpoint1, 20, 1.0, 0.0, 0.0, "sphere1_" + std::to_string(i));
        // viewer->addSphere(cloudpoint2, 20, 0.0, 0.0, 1.0, "sphere2_" + std::to_string(i));
        // viewer->addLine<pcl::PointXYZRGB> (cloudpoint1, cloudpoint2, 255,0,0, "line_" + std::to_string(i));

        feature_cloud1->points.push_back(cloudpoint1);
        feature_cloud2->points.push_back(cloudpoint2);

    }


    boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZRGB, pcl::PointXYZRGB > > estPtr;
    estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB > () );
    Eigen::Affine3f transformation_est;
    // estPtr->estimateRigidTransformation ( *feature_cloud1,
    //                                       *feature_cloud2,
    //                                       transformation_est.matrix());



    // std::cout << "number of features: " << feature_cloud1->points.size() << std::endl;

// std::cout << "Estimating! " << iii << std::endl;
// iii++; 




// RANSAC START


    int max_iterations = 150;
    const int min_support = 5;
    const float inlier_error_threshold = 150.0f;
    const int pcount = feature_cloud1->points.size();

    if (pcount < 10) {
        return Eigen::Affine3f::Identity();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::RNG rng;
    std::vector<int> best_inliers;


    for(int k=0; k < max_iterations; k++) {

        // std::cout << k << std::endl;
        // std::cout << "iterations:" << std::endl;

        random_features1->clear();
        random_features2->clear();


        pcl::PointXYZRGB prevcpoint1;
        pcl::PointXYZRGB prevcpoint2;
        bool skipThisIteration = false;

        //Select random points
        for(int i=0; i < min_support; i++) {
            int idx = rng(pcount);

            pcl::PointXYZRGB & cpoint1 = feature_cloud1->points[idx];
            pcl::PointXYZRGB & cpoint2 = feature_cloud2->points[idx];

            // skip first iteration
            if (i>0) {
                float distance1 = pcl::geometry::distance(cpoint1, prevcpoint1);
                float distance2 = pcl::geometry::distance(cpoint2, prevcpoint2);

                if (abs(distance1 - distance2) > 1000) {
                    skipThisIteration = true;
                    break;
                }
            }



            random_features1->points.push_back(cpoint1);
            random_features2->points.push_back(cpoint2);

            // prevcpoint1 = cpoint1;
            // prevcpoint2 = cpoint1;
        }

        if (skipThisIteration) {
            std::cout << "distance too far!" << std::endl;
            // max_iterations++;
            continue;
        }



        estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB > () );
        Eigen::Affine3f current_transformation;
        estPtr->estimateRigidTransformation ( *random_features1,
                                              *random_features2,
                                              current_transformation.matrix());



        // // Euler angles
        // Eigen::Matrix<float, 3, 3> eigen_R;
        // Eigen::Matrix<float, 3, 1> eigen_T;
        // eigen_R = current_transformation.rotation();
        // eigen_T = current_transformation.translation();
        // Eigen::Vector3f rpy = eigen_R.eulerAngles(0,1,2);

        // std::cout << "Euler angles:" << std::endl
        //          << "roll: " << rpy(0) << std::endl;
        // std::cout << "pitch: " << rpy(1) << std::endl;
        // std::cout << "yaw: " << rpy(2) << std::endl;

        // std::cout <<<< std::endl;

        // if (abs(rpy(0))  > (PI - 0.02)
        //     || abs(rpy(0)) < 0.02
        //     || abs(rpy(2))  > (PI - 0.02)
        //     || abs(rpy(2)) < 0.02
        //     ||  eigen_T(1) > 50) {

        // } else {
        //   max_iterations++;
        //   continue;
        // }


        //Get error
        std::vector<int> inliers;
        for(int i=0; i<pcount; i++) {

            auto & ppt1 = feature_cloud1->points[i];
            auto & ppt2 = feature_cloud2->points[i];
            auto tppt = transformPoint(ppt1, current_transformation);

            double a,b,c, errori = 0;
            a = tppt.x - ppt2.x;
            b = tppt.y - ppt2.y;
            c = tppt.z - ppt2.z;
            errori = sqrt(a*a+b*b+c*c);


            if(errori < inlier_error_threshold) {
                inliers.push_back(i);
            }
        }

        if (inliers.size() < 4) {
            // max_iterations++;
            std::cout << "not enough inliers!!" << std::endl;
            continue;
        }

        if(inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
        }
    }
    std::cout << "Ratio: " <<  ((best_inliers.size() * 100) / pcount) << " Inlier count: " << best_inliers.size() << "/" << pcount << "\n";


    // if (best_inliers.size() < 8) {
    //     return Eigen::Affine3f::Identity();
    // }


    // estimate final transformation with inliers
    random_features1->clear();
    random_features2->clear();

    for(int i=0; i<best_inliers.size(); i++) {
        int idx = best_inliers[i];

        pcl::PointXYZRGB & cpoint1 = feature_cloud1->points[idx];
        pcl::PointXYZRGB & cpoint2 = feature_cloud2->points[idx];

        random_features1->points.push_back(cpoint1);
        random_features2->points.push_back(cpoint2);
    }

    estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB > () );
    Eigen::Affine3f current_transformation;
    estPtr->estimateRigidTransformation ( *random_features1,
                                          *random_features2,
                                          transformation_est.matrix());

    // transform features1 with transformation from ransac and compute ICP
    pcl::transformPointCloud( *random_features1, *random_features1, transformation_est );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_icp (new pcl::PointCloud<pcl::PointXYZRGB>);
    int iterations = 50;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource (random_features1);
    icp.setInputTarget (random_features2);
    icp.setTransformationEpsilon (1e-9);
    icp.setMaxCorrespondenceDistance (50);
    icp.setEuclideanFitnessEpsilon (1);
    icp.setRANSACOutlierRejectionThreshold (1.5);
    icp.align(*transformed_cloud_icp);

    if (icp.hasConverged ()){
        // std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        // std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        auto transformation_icp = icp.getFinalTransformation();
        transformation_est *= transformation_icp;
    }





    // *feature_cloud = *feature_cloud1;




// RANSAC END


// std::cout << "estimated transformation is:  " << transformation_est.matrix() << std::endl;

    return transformation_est;
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
    long long frames_processed = 0;
    long long frame_processing_average_milliseconds = 0;

    // remove this
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());


    Eigen::Affine3f transform_visual_accumulated = Eigen::Affine3f::Identity();
    app::Frame previous_frame;

    while (app::Application::is_running) {
        app::Frame temp_frame;
        bool match_frame = false;
        
        // check if current_frame has changed, if yes, keep it and mark it for processing
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);

            // mark this frame as visited and pass it to processing pipeline
            if (processed_frame.t2_done && !(processed_frame.t3_done)) {
                match_frame = true;
                processed_frame.t3_done = true;
                temp_frame = processed_frame;
            }
        }

        // processing pipeline start
        if (match_frame){
            

            if (frames_processed != 0) {
                auto start = std::chrono::high_resolution_clock::now();


// algorithm    
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform = estimateVisualTransformation(temp_frame, previous_frame, feature_cloud); 
                transform_visual_accumulated = transform_visual_accumulated * transform; 
                temp_frame.transform_visual = transform_visual_accumulated;




                 // time benchmark stop
                auto end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                
            }

            frames_processed++;

            {
                // lock currently processed frame
                std::lock_guard<std::mutex> mutex_guard(matched_frame_mutex);
                matched_frame = temp_frame;
                matched_frame.t3_done = true;
                processed_frame.t3_done = true;
                previous_frame = matched_frame;

            }

        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

    }

    std::cout << "Matching thread frame count: " << frames_processed << " avg time of mapping: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
    std::cout << "Frame Matcher exitting..." << std::endl;
}
