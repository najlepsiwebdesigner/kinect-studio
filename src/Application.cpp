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
#include <pcl/registration/icp.h>

#include <pcl/io/ply_io.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree.h>


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





bool app::Application::is_running = false;

void app::Application::start() {
    is_running = true;

    RobotPose current_robot_pose;
    std::mutex current_robot_pose_mutex;


    struct Options {
        bool show_3D = true;
        bool show2D = true;

    } ;
    Options options;
    options.show_3D = true;
    options.show2D = false;



    CKobuki robot(current_robot_pose);
    try {
        unsigned char *null_ptr(0);
        robot.startCommunication("/dev/cu.usbserial-kobuki_AI02MVQM", true);
    } catch (std::runtime_error e) {
        std::cout << "ERROR: " << e.what() << std::endl;
    }


    bool is_slamming = true;




    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(grabbed_frame, grabbed_frame_mutex, current_robot_pose, current_robot_pose_mutex);
    FrameProcessor frameProcessor(grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);
//    FrameMatcher frameMatcher(processed_frame, processed_frame_mutex);


    std::thread t0([&robot,&is_slamming]() {

//        std::this_thread::sleep_for(std::chrono::milliseconds(10*1000));

//        is_slamming = true;



//        robot.goStraight(0.3);

//        robot.doRotation(PI*2);


//        robot.goStraight(0.3);


//        robot.doRotation(PI);
//        robot.goStraight(0.3);
//        robot.doRotation(PI*2);
//
//        robot.goStraight(0.3);
//        robot.doRotation(PI/2);
//
//        robot.goStraight(0.3);
//        robot.doRotation(PI/2);

//        is_slamming = false;
    });


    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });

//    std::thread t3([&frameMatcher]() {
//       frameMatcher.run();
//    });



// 2D options
    if (options.show2D) {
        // gui initialization
        cv::namedWindow("Depth");
        cv::namedWindow("Video");
        // cv::namedWindow("Thresh");
        // cv::namedWindow("Good Matches");
        // cv::namedWindow("Keypoints");
        // cv::namedWindow("Clahe");
    }

// 3D options
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    if (options.show_3D) {
        viewer->addCoordinateSystem (10.0);
        viewer->initCameraParameters ();
        viewer->setBackgroundColor (0, 0, 0);
        viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
        viewer->setSize(2560, 1920);
        viewer->addArrow(pcl::PointXYZ(0,0,100),pcl::PointXYZ(0,0,0),255,125,125,"arrow");
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> trasformation_estimation;


    long long frame_processing_average_milliseconds = 0;
    long long frames_processed = 0;     

    Frame keyframe;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
    while (is_running) 
    {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);

            // && !processed_frame.descriptors.empty() && !previous_frame.descriptors.empty()

            if (processed_frame.processed) {


                if (options.show_3D) {
                    // realtime 3D rendering
//                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(processed_frame.cloud);

                    if (is_slamming){
                        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    //                    std::cout << "X: " << processed_frame.x << std::endl
    //                              << "Y: " << processed_frame.y << std::endl
    //                              << "theta: " << processed_frame.theta << std::endl;

                        // Define a translation of 2.5 meters on the x axis.
                        transform_2.translation() << processed_frame.y*1000, 0.0,processed_frame.x*1000;

                        // The same rotation matrix as before; theta radians arround Z axis
                        transform_2.rotate (Eigen::AngleAxisf (processed_frame.theta, Eigen::Vector3f::UnitY()));

                        // Print the transformation
    //                        printf ("\nAffine3f\n");
    //                        std::cout << transform_2.matrix() << std::endl;

                        // Executing the transformation
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
                        // You can either apply transform_1 or transform_2; they are the same
                        pcl::transformPointCloud<pcl::PointXYZRGB>(*processed_frame.cloud, *preview_cloud, transform_2);



                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (preview_cloud, 255, 0, 0);
                        if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgb, "sample cloud")) {
                            viewer->addPointCloud<pcl::PointXYZRGB>(preview_cloud, rgb, "sample cloud");

                            // initialize keyframe
                            keyframe = processed_frame;
//                            *keyframe.cloud = *preview_cloud;
                            // pcl::io::savePLYFile("./ply/first.ply", *(processed_frame).cloud);
                        }

//                        boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > kdtree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
//                        kdtree->setInputCloud(preview_cloud);
//                        // distance in [mm]
//                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr difference_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//                        pcl::getPointCloudDifference(*preview_cloud, *preview_cloud, 30, kdtree, *difference_cloud);
//                        std::cout << difference_cloud->points.size() << std::endl;

                        if (frames_processed % 15 == 0 ) {
//                            std::string cloud_id = "sample cloud2_";
//                            cloud_id += std::to_string(frames_processed);
//                            std::cout << cloud_id << std::endl;

                            *model += *preview_cloud;
                            std::cout << "model size:" << model->points.size() << std::endl;

                            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(model);
                            if (!viewer->updatePointCloud<pcl::PointXYZRGB>(model, rgb, "model")) {
//                            if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgb, cloud_id)) {
//
                                viewer->addPointCloud<pcl::PointXYZRGB>(model, rgb, "model");
                            }
                        }
                    }



                    viewer->spinOnce(30);
                }
                // time benchmark stop
                // end = std::chrono::high_resolution_clock::now();
                // auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                // frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                frames_processed++;

                if (options.show2D) {
                    // main CV visualization loop, has to be placed in main thread due to plc and opencv's restrictions
                    cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                    processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

                    cv::imshow("Video", processed_frame.claheMat);
                    cv::imshow("Depth", depthf);
//                    cv::imshow("BA image", processed_frame.baMat);
                }

            }
        } // locked processed_frame

        char c = cvWaitKey(10);
        if (c == 27 || viewer->wasStopped()) {
            std::cout << "char is:" << c << std::endl;
            is_running = false;
            break;
        }
        else if (c == 115) {
            if (is_slamming) {
                is_slamming = false;
            } else {
                is_slamming = true;
            }
            std::cout << "slamming! " << is_slamming << std::endl;
        }




    }




    t1.join();
    t2.join();
//    t3.join();
    t0.join();


        // /// exit main thread
        // std::cout << "Frame count: " << frames_processed << " avg time of transformation estimation: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
        // std::cout << "Main thread thread exitting.." << std::endl;


}

void app::Application::stop() {
    is_running = false;
}