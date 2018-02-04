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



void computeDescriptors(Frame & temp_frame) {
    using namespace cv;
    using namespace cv::xfeatures2d;

    auto & keypoints = temp_frame.keypoints;

    /// ### feature description
    Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
//        Ptr<SURF> extractor = SURF::create();

    extractor->compute(temp_frame.claheMat, keypoints, temp_frame.descriptors);

}



Eigen::Affine3f estimateVisualTransformation(app::Frame & frame1, app::Frame & frame2) {
    const int MAXIMAL_FEATURE_DISTANCE = 20;

    computeDescriptors(frame1);

    if (!frame2.is_predicted) {
        computeDescriptors(frame2);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (frame1.cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (frame2.cloud);

    std::vector<KeyPoint> & keypoints1 = frame1.keypoints;
    std::vector<KeyPoint> & keypoints2 = frame2.keypoints;

    Mat & descriptors1 = frame1.descriptors;
    Mat & descriptors2 = frame2.descriptors;

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // std::cout << "desc1: " << descriptors1.rows << std::endl;
    // std::cout << "desc2: " << descriptors2.rows << std::endl;
    // std::cout << "matches: " << matches.size() << std::endl;

    std::vector<Vector3f> movement_vectors;
    Vector3f average_movement_vector;
    int number_of_matches = 0;


    // create feature point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);


    // create feature cloud for currently processed frame
    for (int i = 0; i < keypoints1.size(); i++) {
        auto cvPoint = keypoints1[i].pt;
        auto & cloudpoint = getCloudPoint(*cloud1, cvPoint.x, cvPoint.y);
        frame1.feature_cloud->points.push_back(cloudpoint);
        frame1.model_indices.push_back(-1);
    }

    // frame1.model_indices.clear();

    for (int i = 0; i < matches.size(); i++) {
        bool is_good_match = false;

        if( matches[i].distance < MAXIMAL_FEATURE_DISTANCE) {
            auto cvPoint1 = keypoints1[matches[i].queryIdx].pt;
            auto & cloudpoint1 = getCloudPoint(*cloud1, cvPoint1.x,cvPoint1.y);

            auto cvPoint2 = keypoints2[matches[i].trainIdx].pt;
            auto & cloudpoint2 = (frame2.is_predicted ? cloud2->points[matches[i].trainIdx] : getCloudPoint(*cloud2, cvPoint2.x,cvPoint2.y));

            if (cloudpoint1.x == 0 && cloudpoint1.y == 0 && cloudpoint1.z == 0) continue;
            if (cloudpoint2.x == 0 && cloudpoint2.y == 0 && cloudpoint2.z == 0) continue;
            if (fabs(cloudpoint1.y - cloudpoint2.y) > 150) continue; 

            if (frame2.is_predicted) {
                auto model_index_predicted = frame2.model_indices[matches[i].trainIdx];
                // std::cout << model_index_predicted << std::endl;
                frame1.model_indices[matches[i].queryIdx] = model_index_predicted;
            }

            feature_cloud1->points.push_back(cloudpoint1);
            feature_cloud2->points.push_back(cloudpoint2);

            frame1.good_feature_matches.push_back(matches[i]);
        }


        // current_feature_cloud->points.push_back()
        // std::cout << "Feature cloud 1 points: " << feature_cloud1->points.size() << std::endl;
        // std::cout << "Feature cloud 1 descriptors: " << feature_cloud1_descriptors.rows << std::endl;

    }


    boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZRGB, pcl::PointXYZRGB > > estPtr;
    estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB > () );
    Eigen::Affine3f transformation_est;


// RANSAC START
    int max_iterations = 1000;
    const int min_support = 5;
    const float inlier_error_threshold = 120.0f;
    const int pcount = feature_cloud1->points.size();

    if (pcount < 6) {
        std::cout << "Not enough features!" << std::endl;
        return Eigen::Affine3f::Identity();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::RNG rng;
    std::vector<int> best_inliers;


    for(int k = 0; k < max_iterations; k++) {

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
            // std::cout << "distance too far!" << std::endl;
            // max_iterations++;
            continue;
        }



        estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB > () );
        Eigen::Affine3f current_transformation;
        estPtr->estimateRigidTransformation ( *random_features1,
                                              *random_features2,
                                              current_transformation.matrix());

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
            // std::cout << "not enough inliers!!" << std::endl;
            continue;
        }

        if(inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
        }
    }
    // std::cout << "Ratio: " <<  ((best_inliers.size() * 100) / pcount) << " Inlier count: " << best_inliers.size() << "/" << pcount << "\n";


    // if (best_inliers.size() < 8) {
    //     return Eigen::Affine3f::Identity();
    // }


    // estimate final transformation with inliers
    random_features1->clear();
    random_features2->clear();

    for(int i=0; i < best_inliers.size(); i++) {
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
    int iterations = 100;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource (random_features1);
    icp.setInputTarget (random_features2);
    icp.setTransformationEpsilon (1e-9);
    icp.setMaxCorrespondenceDistance (200);
    icp.setEuclideanFitnessEpsilon (1);
    icp.setRANSACOutlierRejectionThreshold (1.3);
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

    // // remove this
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());


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
                temp_frame = processed_frame;
            }
        }

        // processing pipeline start
        if (match_frame){
            

            if (frames_processed != 0) {
                auto start = std::chrono::high_resolution_clock::now();

                {
                    std::lock_guard<std::mutex> mutex_guard(map_model_mutex);
                    
                    Frame tmp_frame;
                    if (map_model.is_ready) {
                        tmp_frame = map_model.getPredictedFrame();
                    } else {
                        tmp_frame = previous_frame;
                    }
                
                    // algorithm    
                    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                    transform = estimateVisualTransformation(temp_frame, tmp_frame);
                    transform_visual_accumulated = transform_visual_accumulated * transform; 
                    temp_frame.transform_visual = transform_visual_accumulated;

                    // pcl::transformPointCloud<pcl::PointXYZRGB>(*temp_frame.feature_cloud, *temp_frame.cloud, transform);

                    map_model.insertFrame(temp_frame);
                }

                 // time benchmark stop
                auto end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                
            }


            frames_processed++;

            // std::cout << "Matcher frames processed: " << frames_processed << std::endl;

            {
                // lock currently processed frame
                std::lock_guard<std::mutex> mutex_guard(matched_frame_mutex);
                std::lock_guard<std::mutex> mutex_guard2(processed_frame_mutex);
                matched_frame = temp_frame;
                // std::cout << "frame matcher: " << temp_frame.x << std::endl;
                matched_frame.t3_done = true;
                processed_frame.t3_done = true;
                previous_frame = matched_frame;

            }

        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

    }

    std::cout << "Matching thread frame count: " << frames_processed << " avg time of mapping: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
    std::cout << "Frame Matcher exitting..." << std::endl;
}
