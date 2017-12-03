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
#include <pcl/common/geometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;
using namespace std::chrono;
using namespace pcl::visualization;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;
using namespace app;



pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
bool app::Application::is_running = true;
int saving_counter = 0;
cv::Mat current_rgb_mat;




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
    if (event.getKeySym() == "i" && event.keyDown()) {

        pcl::io::savePLYFileBinary("model.ply", *model);
        std::cout << "Saving model!" << std::endl;
    }
//    if (event.getKeySym() == "i" && event.keyDown()) {
//
//        vector<int> compression_params;
//        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//        compression_params.push_back(1);
//
//        pcl::io::savePLYFileBinary(std::to_string(saving_counter) + "-frame.ply", *preview_cloud);
//        cv::imwrite(std::to_string(saving_counter) + "-rgb.png", current_rgb_mat, compression_params);
//        std::cout << "Saving frame!" << std::endl;
//        saving_counter++;
//    }
}



void app::Application::start(int argc, char** argv) {

    // application options
    static Options options;

    if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
        printUsage(argv[0]);
        return;
    } else if (pcl::console::find_argument(argc, argv, "-r") >= 0) {
        options.is_recording = true;
        options.is_slamming = false;
        options.show2D = false;
        options.show_3D = true;
        std::cout << "Now i should record frames" << std::endl;
    } else if (pcl::console::find_argument(argc, argv, "-s") >= 0) {
        options.is_slamming = true;
        options.show2D = false;
        options.show_3D = true;
        options.is_recording = false;
        std::cout << "Now I should do online SLAM" << std::endl;
    }
    if (pcl::console::find_argument(argc, argv, "-o") >= 0) {
        options.offline = true;
        std::cout << "Now I will do offline slam from selected path as input argument" << std::endl;
    }

    // overridden application options
    options.show2D = false;
    options.show_3D = true;
    options.is_slamming = true;
    // options.is_recording = true;
    // options.offline = true;



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

    Frame matched_frame;
    std::mutex matched_frame_mutex;

    FrameGenerator frameGenerator(options, grabbed_frame, grabbed_frame_mutex, current_robot_pose,
                                  current_robot_pose_mutex);
    FrameProcessor frameProcessor(options, grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);
    FrameMatcher frameMatcher(options, processed_frame, processed_frame_mutex, matched_frame, matched_frame_mutex);

    // control robot here
    std::thread t0([&robot]() {

    });

    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });

    std::thread t3([&frameMatcher]() {
        frameMatcher.run();
    });



    // 2D options, gui initialization
    if (options.show2D) {
        cv::namedWindow("Depth");
        cv::namedWindow("Video");
    }

    // 3D options, initialize 3d viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    if (options.show_3D) {
        viewer->addCoordinateSystem(10.0);
        viewer->initCameraParameters();
        viewer->setBackgroundColor(0, 0, 0);
        viewer->setCameraPosition(0, 0, -1000, 0, -1, 0);
        viewer->setSize(2560, 1920);
    }
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) viewer.get());


    // time benchmark start
    long long frame_processing_average_milliseconds = 0;
    long long frames_processed = 0;

    std::vector<Eigen::Affine3f> camera_poses_vector;
    std::vector<std::string> sphere_names;
    // Frame previous_frame;


    Eigen::Affine3f transform_visual_accumulated = Eigen::Affine3f::Identity();
    // std::cout << transform_visual_accumulated.matrix() << std::endl;

    std::vector<std::string> feature_sphere_names;

    while (is_running) {


        // app::Frame temp_frame;
        bool visualize_frame = false;
            Frame temp_frame;
        
        // check if current_frame has changed, if yes, keep it and mark it for processing
        {
            std::lock_guard<std::mutex> mutex_guard(matched_frame_mutex);

            // mark this frame as visited and pass it to processing pipeline
            if (matched_frame.t3_done) {
                visualize_frame = true;
                matched_frame.t4_done = true;
                 temp_frame = matched_frame;

            }
        }


        Eigen::Affine3f transform;

        // transform = temp_frame.transform_odometry;
        transform = temp_frame.transform_visual;

  

        pcl::PointXYZRGB initial_camera_pose(0, 0, 0);
        pcl::PointXYZRGB camera_pose;
        camera_pose = pcl::transformPoint(initial_camera_pose, transform);

        pcl::transformPointCloud<pcl::PointXYZRGB>(*temp_frame.cloud, *preview_cloud, transform);


        // this computes and updates world model
        if (options.is_slamming) { 

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
            // Create the filtering object
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (preview_cloud);
            sor.setLeafSize (40, 40, 40);
            sor.filter (*cloud_filtered);
            *model += *cloud_filtered;

        } 

        if (options.show_3D) {
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



            // preview cloud visualization
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbc(preview_cloud);
            if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "current_cloud")) {
                viewer->addPointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "current_cloud");
            }

        }

        if (options.show2D) {
            // // main CV visualization loop, has to be placed in main thread due to plc and opencv's restrictions
            // cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
            // temp_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

            // cv::imshow("Video", temp_frame.claheMat);
            // cv::imshow("Depth", depthf);
            // // cv::imshow("BA", processed_frame.baMat);
        }

        
        frames_processed++;

        viewer->spinOnce(3);

        char c = cvWaitKey(10);
        if (c == 27 || viewer->wasStopped()) {
            std::cout << "char is:" << c << std::endl;
            is_running = false;
            break;
        }
        else if (c == 115) {
            options.is_slamming = !options.is_slamming;
            std::cout << "slamming! " << options.is_slamming << std::endl;
        }


    }

    is_running = false;

    t0.join();
    t1.join();
    t2.join();
    t3.join();

    viewer->close();


    pcl::io::savePLYFileBinary("model.ply", *model);
    std::cout << "Saving model!" << std::endl;

    // exit main thread
    std::cout << "Main thread exitting.." << std::endl;

}

void app::Application::stop() {
    is_running = false;
}
