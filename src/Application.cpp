//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"

#include <math.h>
//#include <map>
#include <pcl/common/distances.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/transformation_estimation_svd.h>
// #include <pcl/registration/icp.h>

#include <pcl/io/ply_io.h>
// #include <pcl/segmentation/segment_differences.h>
//#include <pcl/kdtree/kdtree.h>

#include <boost/math/special_functions/round.hpp>

// #include <pcl/octree/octree.h>
// #include <pcl/octree/octree_impl.h>

//#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/console/parse.h>
//
//#include <pcl/visualization/point_cloud_handlers.h>
//#include <pcl/visualization/common/common.h>
//#include <vtkRenderWindow.h>
//#include <vtkCubeSource.h>
//#include <vtkDataSetMapper.h>
//#include <vtkAppendPolyData.h>

// #include <pcl/surface/convex_hull.h>
// 
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


using namespace std;
using namespace std::chrono;
using namespace pcl::visualization;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;
using namespace app;



pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
bool app::Application::is_running = true;
int saving_counter = 0;
cv::Mat current_rgb_mat;



pcl::PointXYZRGB & getCloudPoint(pcl::PointCloud<pcl::PointXYZRGB> & my_pcl,  int x, int y) {
    int arrayPosition = y*my_pcl.width + x * 1;

    pcl::PointXYZRGB & cloudPoint = my_pcl.points[arrayPosition];

    return cloudPoint;
}





Eigen::Affine3f estimateVisualTransformation(app::Frame & frame1, app::Frame & frame2) {
  cv::Mat img1 = frame1.rgbMat;
  cv::Mat img2 = frame2.rgbMat;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (frame1.cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (frame2.cloud);


  const int MAXIMAL_FEATURE_DISTANCE = 10;
  /// ### feature detection ± 13 ms
  //         Ptr<SURF> detector = SURF::create( 100,4,1,false,false );
          // Ptr<AKAZE> detector = AKAZE::create();
  Ptr<ORB> detector = ORB::create(1500);
  //         Ptr<SIFT> detector = SIFT::create();
  // Ptr<MSER> detector = MSER::create();
  //         Ptr<BRISK> detector = BRISK::create();
  //         Ptr<KAZE> detector = KAZE::create();
          // Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();

  std::vector<KeyPoint> keypoints1,keypoints2;
  detector->detect( img1, keypoints1);
  detector->detect( img2, keypoints2);

  /// ### feature description
  Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
  // Ptr<SURF> extractor = SURF::create();
  Mat descriptors1, descriptors2;

  extractor->compute(img1, keypoints1, descriptors1);   
  extractor->compute(img2, keypoints2, descriptors2);   

  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);
  
  std::cout << "number of matches: " << matches.size() << std::endl;

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

  std::cout << "number of good matches: " << matches.size() << std::endl;


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
    if (fabs(cloudpoint1.y - cloudpoint2.y) > 200) continue;
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

// RANSAC START


    const int max_iterations = 100;
    const int min_support = 6;
    const float inlier_error_threshold = 40.0f;
    const int pcount = feature_cloud1->points.size();

    if (pcount < 10) {
        return Eigen::Affine3f::Identity();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_features2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::RNG rng;    
    std::vector<int> best_inliers;


    for(int k=0; k<max_iterations; k++) {

        random_features1->clear();
        random_features2->clear();

        //Select random points
        for(int i=0; i<min_support; i++) {
            int idx = rng(pcount);
            pcl::PointXYZRGB & cpoint1 = feature_cloud1->points[idx];
            pcl::PointXYZRGB & cpoint2 = feature_cloud2->points[idx];

            random_features1->points.push_back(cpoint1);
            random_features2->points.push_back(cpoint2);
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



        if(inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
        }
    }
    std::cout << "Inlier count: " << best_inliers.size() << "/" << pcount << "\n";


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






// RANSAC END


// std::cout << "estimated transformation is:  " << transformation_est.matrix() << std::endl;

  return transformation_est;
}




void transformationDifference(const Eigen::Affine3f & transform1, const Eigen::Affine3f & transform2, double & angle, double & length) {
    auto translation1 = transform1.translation();
    auto translation2 = transform2.translation();

    auto rotation1 = transform1.rotation();
    auto rotation2 = transform2.rotation();

    auto finalRotation = rotation1 * rotation2.inverse();

    Eigen::AngleAxisf newAngleAxis(finalRotation);

    angle = newAngleAxis.angle();
    auto result = translation1 - translation2;
    length = result.norm();
}






void printUsage (const char* progName)
{
    std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-h           this help\n"
              << "-r           Record frames during capture\n"
              << "-s           Do odometry slam\n"
              << "-o           Use offline data instead of live stream\n"
              << "\n\n";
}


// save whole map on key press, bugged @TODO!
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
//    if (event.getKeySym() == "i" && event.keyDown()) {
//
//        pcl::io::savePLYFileBinary("model.ply", *model);
//        std::cout << "Saving model!" << std::endl;
//    }
    if (event.getKeySym() == "i" && event.keyDown()) {

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(1);

        pcl::io::savePLYFileBinary(std::to_string(saving_counter) + "-frame.ply", *preview_cloud);
        cv::imwrite(std::to_string(saving_counter) + "-rgb.png", current_rgb_mat, compression_params);
        std::cout << "Saving frame!" << std::endl;
        saving_counter++;
    }
}



void app::Application::start(int argc, char** argv) {

    // application options
    static Options options;

    if (pcl::console::find_argument (argc, argv, "-h") >= 0) {
        printUsage (argv[0]);
        return;
    } else if (pcl::console::find_argument (argc, argv, "-r") >= 0) {
        options.is_recording = true;
        options.is_slamming = false;
        options.show2D = false;
        options.show_3D = true;
        std::cout << "Now i should record frames" << std::endl;
    } else if (pcl::console::find_argument (argc, argv, "-s") >= 0) {
        options.is_slamming = true;
        options.show2D = false;
        options.show_3D = true;
        options.is_recording = false;
        std::cout << "Now I should do online SLAM" << std::endl;

        if (pcl::console::find_argument(argc, argv, "-o") >= 0) {
            options.offline = true;
            std::cout << "Now I will do offline slam from selected path as input argument" << std::endl;
        }
    }

    // overridden application options
    options.show2D = false;
    options.show_3D = true;
    options.is_slamming = true;
    options.is_recording = false;
    options.offline = true;



    // robot initial setup, setup shared variables which are used for propagation of current pose elsewhere
    RobotPose current_robot_pose;
    std::mutex current_robot_pose_mutex;

    CKobuki robot(current_robot_pose);
    try {
        if (!options.offline) {
            robot.startCommunication("/dev/cu.usbserial-kobuki_AI02MVQM", true);
        }
    } catch (std::runtime_error e) {
        std::cout << "ERROR: " << e.what() << std::endl;
    }


    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(options, grabbed_frame, grabbed_frame_mutex, current_robot_pose, current_robot_pose_mutex);
    FrameProcessor frameProcessor(options, grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);
//    FrameMatcher frameMatcher(processed_frame, processed_frame_mutex);

    // control robot here
    std::thread t0([&robot]() {

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



    // 2D options, gui initialization
    if (options.show2D) {
        cv::namedWindow("Depth");
        cv::namedWindow("Video");
    }

    // 3D options, initialize 3d viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    if (options.show_3D) {
        viewer->addCoordinateSystem (10.0);
        viewer->initCameraParameters ();
        viewer->setBackgroundColor (0, 0, 0);
        viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
        viewer->setSize(2560, 1920);
    }
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());


    // time benchmark start
    long long frame_processing_average_milliseconds = 0;
    long long frames_processed = 0;

    std::vector<Eigen::Affine3f> camera_poses_vector;
    std::vector<std::string> sphere_names;
    Frame previous_frame;

    Eigen::Affine3f transform_visual_accumulated = Eigen::Affine3f::Identity();
    // std::cout << transform_visual_accumulated.matrix() << std::endl;

    std::vector<std::string> feature_sphere_names;

    while (is_running)
    {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);
 

            // grab currently pre-processed frame
            if (processed_frame.processed) {


                auto start = std::chrono::high_resolution_clock::now();


                Eigen::Affine3f transform;

                // skip first frame transformation estimation
                if (previous_frame.processed) {
                    *preview_cloud = *processed_frame.cloud;

                    transform = estimateVisualTransformation(processed_frame, previous_frame);
                    // visual odometry has to be integrated
                    transform_visual_accumulated = transform_visual_accumulated * transform;

                    
                
                }


// Transformation estimation!!!

                // double diffAngle, diffLength;
                // transformationDifference(transform_visual_accumulated, processed_frame.transform_odometry, diffAngle, diffLength);
                // if (diffLength > 150 || diffAngle >  (15 * PI/180)) {
                    transform = processed_frame.transform_odometry;
                    // transform_visual_accumulated = processed_frame.transform_odometry;
                //     std::cout << "fail!" << std::endl;
                // } else {
                    // transform = transform_visual_accumulated;    
                // }
                // std::cout << transform.matrix() << std::endl;
                
                
                
                

                

                pcl::PointXYZRGB initial_camera_pose(0,0,0);
                pcl::PointXYZRGB camera_pose;
                camera_pose = pcl::transformPoint(initial_camera_pose,transform);


                pcl::transformPointCloud<pcl::PointXYZRGB>(*processed_frame.cloud, *preview_cloud, transform); 
                




                // this computes and updates world model
                if (options.is_slamming) {

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
                    // Create the filtering object
                    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                    sor.setInputCloud (preview_cloud);
                    sor.setLeafSize (20, 20, 20);
                    sor.filter (*cloud_filtered);


                    *model += *cloud_filtered;

                    
                }

                // time benchmark stop
                auto end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                frames_processed++;

                if (options.show_3D) {

                    // camera posietions!!!
                    // remove all previous spheres
                    for (const auto & s_name : sphere_names) {
                        viewer->removeShape(s_name);
                    }
                    sphere_names.clear();

                    int i = 0;
                    for (const auto & pose: camera_poses_vector) {

                        double angle, length;
                        transformationDifference(pose, transform, angle, length);

                        // std::cout << "angle: " << angle << " length: " << length << std::endl;

                        // comparision of 3d transformations from eigen, first number is translation threshold in [mm],
                        // second is angle threshold in [degrees]
                        if (length < 150 && angle <  (45 * PI/180)) {
                            pcl::PointXYZRGB cam_pose(0,0,0);
                            cam_pose = pcl::transformPoint(cam_pose,pose);

                            viewer->addSphere(cam_pose, 20, 255, 0, 0,std::to_string(frames_processed) + std::to_string(i));

                            sphere_names.push_back(std::to_string(frames_processed) + std::to_string(i));
                        }
                        i++;
                    }
                    camera_poses_vector.push_back(transform);

                    // add current camera position
                    std::string sphere_name = "sphere";
                    sphere_name += std::to_string(frames_processed);
                    sphere_names.push_back(sphere_name);
                    viewer->addSphere(camera_pose, 20, 255, 255, 255, sphere_name);

                    // end camera poses!





                    // visualize world model
                   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(model);
                   // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(model, "x");
                   if (!viewer->updatePointCloud<pcl::PointXYZRGB>(model, rgb, "model")) {
                       viewer->addPointCloud<pcl::PointXYZRGB>(model, rgb, "model");
                   }
                   std::cout << "Model size:" <<  model->points.size() << std::endl;

                    // if (previous_frame.processed) {
                    //     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbc(previous_frame.cloud);
                    //     if (!viewer->updatePointCloud<pcl::PointXYZRGB>(previous_frame.cloud, rgbc, "previous_cloud")) {
                    //         viewer->addPointCloud<pcl::PointXYZRGB>(previous_frame.cloud, rgbc, "previous_cloud");
                    //     }
                    // }

                    // preview cloud visualization
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbc(preview_cloud);
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "current_cloud")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "current_cloud");
                    }

                }


                if (options.show2D) {
                    // main CV visualization loop, has to be placed in main thread due to plc and opencv's restrictions
                    cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                    processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

                    cv::imshow("Video", processed_frame.claheMat);
                    cv::imshow("Depth", depthf);
                }
            }


            previous_frame = processed_frame;
        } // locked processed_frame

        viewer->spinOnce(5);

        char c = cvWaitKey(10);
        if (c == 27 || viewer->wasStopped()) {
            std::cout << "char is:" << c << std::endl;
            is_running = false;
            break;
        }
        else if (c == 115) {
            if (options.is_slamming) {
                options.is_slamming = false;
            } else {
                options.is_slamming = true;
            }
            std::cout << "slamming! " << options.is_slamming << std::endl;
        }


    }

    t0.join();
    t1.join();
    t2.join();

    viewer->close();

    // exit main thread
    std::cout << "Frame count: " << frames_processed << " avg time of mapping: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
    std::cout << "Main thread exitting.." << std::endl;
    is_running = false;
}

void app::Application::stop() {
    is_running = false;
}