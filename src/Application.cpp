//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"

#include <map>
#include <pcl/common/distances.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>



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
    // gui initialization
    cv::namedWindow("Depth");
    cv::namedWindow("Video");
    cv::namedWindow("Thresh");
    cv::namedWindow("Good Matches");
    cv::namedWindow("Keypoints");
    cv::namedWindow("Clahe");
    is_running = true;




    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(grabbed_frame, grabbed_frame_mutex);
    FrameProcessor frameProcessor(grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);



    // stuff for framematcher thread
    Frame previous_frame;





    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });



    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (10.0);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
    viewer->setSize(640, 480);

    viewer->addArrow(pcl::PointXYZ(0,0,100),pcl::PointXYZ(0,0,0),255,125,125,"arrow");


    while (is_running) {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);


            if (processed_frame.processed) {



                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB> ());



                if (!processed_frame.descriptors.empty() && !previous_frame.descriptors.empty()) {

                    cv::Mat img_keypoints = processed_frame.claheMat.clone();//cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));//processed_frame.claheMat.clone();
                    //-- Step 3: Matching descriptor vectors using Bruteforce matcher
                    cv::BFMatcher matcher(cv::NORM_HAMMING);
                    std::vector<cv::DMatch> matches;
                    matcher.match(processed_frame.descriptors, previous_frame.descriptors, matches);


                    // maps 3D point distances to matches, int is
                    std::map<int, double> distances3DMap;
                    std::vector<double> distances3D;

                    double max_dist = 0;
                    double min_dist = 1000000;

                    //-- Quick calculation of max and min distances between keypoints
                    for (int i = 0; i < matches.size(); i++) {
                        double dist = matches[i].distance;
                        if (dist < min_dist) min_dist = dist;
                        if (dist > max_dist) max_dist = dist;
                    }

//                    printf("-- Max dist : %f \n", max_dist);
//                    printf("-- Min dist : %f \n", min_dist);


                    for (int i = 0; i < matches.size(); i++) {
                        if (matches[i].distance < max_dist / 2) {
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
//                    std::cout << "median 3D distance movement: " << medianDistanceMovement << std::endl;

                    std::vector<cv::DMatch> good_matches;

                    for (int i = 0; i < matches.size(); i++) {
                        if (distances3DMap[i] != 0 && distances3DMap[i] < medianDistanceMovement) {
                            good_matches.push_back(matches[i]);
                        }
                    }

                    // red is good
                    for (int i = 0; i < good_matches.size(); i++) {
                        cv::circle(img_keypoints, processed_frame.keypoints[good_matches[i].queryIdx].pt, 1,
                                   cv::Scalar(0, 0, 255));
                        makePointRed(*processed_frame.cloud, processed_frame.keypoints[good_matches[i].queryIdx].pt.x,
                                     processed_frame.keypoints[good_matches[i].queryIdx].pt.y);


                        // redundant
                        auto currentFrameCVPoint = processed_frame.keypoints[good_matches[i].queryIdx].pt;
                        auto previousFrameCVPoint = previous_frame.keypoints[good_matches[i].trainIdx].pt;
                        pcl::PointXYZRGB  currentCloudPoint = getCloudPoint(*(processed_frame).cloud, currentFrameCVPoint.x,
                                                                                 currentFrameCVPoint.y);
                        pcl::PointXYZRGB previousCloudPoint = getCloudPoint(*(processed_frame).cloud, previousFrameCVPoint.x,
                                                                previousFrameCVPoint.y);


                        cloud_in->push_back(currentCloudPoint);
                        cloud_out->push_back(previousCloudPoint);


                        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> TESVD;
                        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB>::Matrix4 transformation2;
                        TESVD.estimateRigidTransformation (*cloud_in,*cloud_out,transformation2);

//                        std::cout << "transformation: " << transformation2 << std::endl;

//                        pcl::transformPointCloud(*cloud_out,*cloud_out,transformation2 , true);

//                        viewer->addArrow(pcl::PointXYZ(0,0,100),pcl::PointXYZ(0,0,0),255,125,125,"arrow");
//                        viewer->removeShape("arrow");
//
//                        pcl::PointXYZRGB arrowDestination(0,0,100);
//                        pcl::transformPoint(arrowDestination,transformation2);
//
//                        viewer->addArrow(arrowDestination,pcl::PointXYZRGB(0,0,0),255,125,125,"arrow");

//                    std::cout << "number of good matches: " << good_matches.size() << std::endl;

                        cv::imshow("Keypoints", img_keypoints);
                    }


//                    std::cout << "transformation will be estimated between: " << cloud_in->points.size() << " and"
//                              << cloud_out->points.size() << std::endl;


                }



                // main visualization loop, has to be placed in main thread due to plc and opencv's restrictions
                cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

//                cv::imshow("Video", processed_frame.rgbMat);
//                cv::imshow("Depth", depthf);

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(processed_frame.cloud);
                if (!viewer->updatePointCloud<pcl::PointXYZRGB> (processed_frame.cloud,rgb, "sample cloud")) {
                    viewer->addPointCloud<pcl::PointXYZRGB> (processed_frame.cloud, rgb, "sample cloud");
                }
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_handler (cloud_in, 255, 255, 255);
//                if (!viewer->updatePointCloud<pcl::PointXYZRGB> (cloud_in, cloud_in_color_handler,"in cloud")) {
//                    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_in,cloud_in_color_handler, "in cloud");
//                }
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_out_color_handler (cloud_out, 255, 0, 0);
//                if (!viewer->updatePointCloud<pcl::PointXYZRGB> (cloud_out,cloud_out_color_handler, "out cloud")) {
//                    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_out, cloud_out_color_handler, "out cloud");
//                }
                viewer->spinOnce (1);


                previous_frame = processed_frame;
            }



        }


        if (cvWaitKey(1) >= 0) {
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