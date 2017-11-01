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
//#include <pcl/kdtree/kdtree.h>

#include <boost/math/special_functions/round.hpp>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

//#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/console/parse.h>
//
//#include <pcl/visualization/point_cloud_handlers.h>
//#include <pcl/visualization/common/common.h>
//#include <vtkRenderWindow.h>
//#include <vtkCubeSource.h>
//#include <vtkDataSetMapper.h>
//#include <vtkAppendPolyData.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
bool app::Application::is_running = true;
int saving_counter = 0;


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

        pcl::io::savePLYFileBinary(std::to_string(saving_counter) + "-frame.ply", *preview_cloud);
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
    options.is_slamming = false;
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
//        viewer->addArrow(pcl::PointXYZ(0,0,100),pcl::PointXYZ(0,0,0),255,125,125,"arrow");
    }
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());


    // world model and temporary octree used for comparision with previous frame - bugged
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> world_octree(0.05);
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(5);
    // indices of collision points between previous frame and current frame
    std::vector<int> newPointIdxVector;

    // time benchmark start
    long long frame_processing_average_milliseconds = 0;
    long long frames_processed = 0;

    std::vector<Eigen::Affine3f> camera_poses_vector;
    std::vector<std::string> sphere_names;

    while (is_running)
    {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);


            // grab currently pre-processed frame
            if (processed_frame.processed) {

                auto start = std::chrono::high_resolution_clock::now();

                // Compute transformation
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << processed_frame.y*1000, 0.0,processed_frame.x*1000;
                transform.rotate (Eigen::AngleAxisf (processed_frame.theta, Eigen::Vector3f::UnitY()));

                // Transform cloud and compute camera pose
                pcl::transformPointCloud<pcl::PointXYZRGB>(*processed_frame.cloud, *preview_cloud, transform);
                pcl::PointXYZRGB initial_camera_pose(0,0,0);
                pcl::PointXYZRGB camera_pose;
                camera_pose = pcl::transformPoint(initial_camera_pose,transform);

                // this computes and updates model
                if (options.is_slamming) {

                    // load new frame into octree collision object and compute overlap
                    octree.switchBuffers();
                    octree.setInputCloud(preview_cloud);
                    octree.addPointsFromInputCloud();
                    newPointIdxVector.clear();


                    // time benchmark stop
                    auto end = std::chrono::high_resolution_clock::now();
                    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                    frames_processed++;



                    // Get vector of point indices from octree voxels which did not exist in previous buffer
                    octree.getPointIndicesFromNewVoxels (newPointIdxVector, 20);

                    // if there is some amount of points missing, add current frame to scene
                    if (newPointIdxVector.size() > 20000) {
                        world_octree.setInputCloud(preview_cloud);
                        world_octree.addPointsFromInputCloud();

//                            std::cout << "Model updated!" << std::endl;
                    }

                    // re-render world-model octree as point cloud, using centeroids as new points
                    auto tree_it = world_octree.begin(9);
                    auto tree_it_end = world_octree.end();


                    model->points.clear();
                    for (; tree_it != tree_it_end; ++tree_it)
                    {
                        Eigen::Vector3f voxel_min, voxel_max;
                        world_octree.getVoxelBounds(tree_it, voxel_min, voxel_max);
                        pcl::PointXYZRGB pt;
                        pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
                        pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
                        pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;

//                        pt.r = 0;
//                        pt.g = 140;
//                        pt.b = 255;
                        model->points.push_back(pt);
                    }
                }



                if (options.show_3D) {

                    // remove all previous spheres
                    for (const auto & s_name : sphere_names) {
                        viewer->removeShape(s_name);
                    }
                    sphere_names.clear();


                    int i = 0;
                    for (const auto & pose: camera_poses_vector) {

                        auto translation1 = pose.translation();
                        auto translation2 = transform.translation();

                        auto rotation1 = pose.rotation();
                        auto rotation2 = transform.rotation();

                        auto finalRotation = rotation1 * rotation2.inverse();

                        Eigen::AngleAxisf newAngleAxis(finalRotation);

                        auto angle = newAngleAxis.angle();
                        auto result = translation1 - translation2;

                        // comparision of 3d transformations from eigen, first number is translation threshold in [mm],
                        // second is angle threshold in [degrees]
                        if (result.norm() < 150 && angle <  (45 * PI/180)) {
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

////                        std::cout << "size of difference: " << newPointIdxVector.size() << std::endl;
//                        // highlight collision points in current frame
//                        for (auto it = preview_cloud->points.begin(); it != preview_cloud->points.end(); ++it) {
//                            it->r = 0;
//                            it->g = 0;
//                            it->b = 255;
//                        }

                    // visualize world model
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(model, "x");
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(model, rgb, "model")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(model, rgb, "model");
                    }



//                        for (std::vector<int>::iterator it = newPointIdxVector.begin();
//                             it != newPointIdxVector.end(); ++it) {
//                            preview_cloud->points[*it].r = 255;
//                            preview_cloud->points[*it].g = 0;
//                            preview_cloud->points[*it].b = 0;
//                        }

                    // current cloud visualization
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbb(preview_cloud);
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgbb, "sample cloud")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(preview_cloud, rgbb, "sample cloud");
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
//    t3.join();

    viewer->close();
    // exit main thread
    std::cout << "Frame count: " << frames_processed << " avg time of transformation of point cloud: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
    std::cout << "Main thread exitting.." << std::endl;
    is_running = false;
}

void app::Application::stop() {
    is_running = false;
}