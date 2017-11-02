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


#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>


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




// ### DETECT FLOOR AND COMPUTE TRANSFORMATION!

                // this is what we are lookign for
                Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();


                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
                int depth_idx = 0;
                for (int v = 0; v < 480; ++v)
                {
                    for (int u = 0; u < 640; ++u, ++depth_idx)
                    {
                        if (v > 380 ) {
                            pcl::PointXYZRGB & pt = processed_frame.cloud->points[depth_idx];
//                            pt.r = 0;
//                            pt.g = 255;
//                            pt.b = 0;
                            if (pt.z < 2000) {
                                cropped_cloud->points.push_back(pt);
                            }
                        }
                    }
                }


//                std::cout << cropped_cloud->points.size() << std::endl;
//                    *preview_cloud = *cropped_cloud;



                // segment plane and display it
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZRGB> seg;
                // Optional
                seg.setOptimizeCoefficients (false);
                // Mandatory
                seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setMaxIterations (5000);

                seg.setDistanceThreshold (0.5);

                seg.setInputCloud (cropped_cloud);

                //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
                Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0); //y axis
                seg.setAxis(axis);
                seg.setEpsAngle(  1.0f * (PI/180.0f) ); // plane can be within 2 degrees of X-Z plane

                seg.segment (*inliers, *coefficients);


                pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tilted_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

                if (inliers->indices.size () > 2000)
                {

                    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
                    proj.setModelType (pcl::SACMODEL_PLANE);
                    proj.setInputCloud(processed_frame.cloud);
                    proj.setModelCoefficients (coefficients);
                    proj.filter (*ground_cloud);


                    // plane model ax + by + cz + d = 0
//                        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                  << coefficients->values[1] << " "
//                                  << coefficients->values[2] << " "
//                                  << coefficients->values[3] << std::endl;
//
//                        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//
//                        for (size_t i = 0; i < inliers->indices.size (); ++i) {
//                            cropped_cloud->points[inliers->indices[i]].r = 255;
//                            cropped_cloud->points[inliers->indices[i]].g = 0;
//                            cropped_cloud->points[inliers->indices[i]].b = 0;
//                        }




                    // vypocitaj transformaciu medzi rovinami
                    // https://stackoverflow.com/questions/32299903/rotation-and-transformation-of-a-plane-to-xy-plane-and-origin-in-pointcloud
                    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

                    floor_plane_normal_vector[0] = coefficients->values[0];
                    floor_plane_normal_vector[1] = coefficients->values[1];
                    floor_plane_normal_vector[2] = coefficients->values[2];

                        std::cout << floor_plane_normal_vector << std::endl;

                    xy_plane_normal_vector[0] = 0.0;
                    xy_plane_normal_vector[1] = 1.0;
                    xy_plane_normal_vector[2] = 0.0;

//                        std::cout << xy_plane_normal_vector << std::endl;
//                    floor_plane_normal_vector.normalize();
                    rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
//                    rotation_vector.normalize();
                        std::cout << "Rotation Vector: "<< rotation_vector << std::endl;

                    long double theta = 0;
//                    std::cout << rotation_vector[2] << std::endl;
                    // https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors#comment69460296_16544330
                    if (rotation_vector[2] >= 0) {
                        theta = atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));
                    } else if (rotation_vector[2] < 0){
                        theta = PI - atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));
                    }

//                    theta = theta * -1;
                    std::cout << "Theta: " << theta << std::endl;


//                        transform_2.translation() << transform.translation();
                    transform_2.rotate (Eigen::AngleAxisf (theta, rotation_vector));

//                        std::cout << "Transformation matrix: " << std::endl << transform_2.matrix() << std::endl;

                }

                *cropped_cloud = *ground_cloud; // *cropped_cloud +


// ### GROUND CLOUD COMPUTATION END




                // Compute transformation
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << processed_frame.y*1000, 0.0,processed_frame.x*1000;
                transform.rotate (Eigen::AngleAxisf (processed_frame.theta, Eigen::Vector3f::UnitY()));

                // Transform cloud and compute camera pose
//                pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_aligned_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//                pcl::transformPointCloud (*processed_frame.cloud, *ground_aligned_cloud, transform_2.inverse());


                pcl::transformPointCloud<pcl::PointXYZRGB>(*processed_frame.cloud, *preview_cloud, transform_2 * transform);

                pcl::transformPointCloud (*cropped_cloud, *ground_cloud, transform_2 * transform );



                pcl::PointXYZRGB initial_camera_pose(0,0,0);
                pcl::PointXYZRGB camera_pose;
                camera_pose = pcl::transformPoint(initial_camera_pose,transform);


                // this computes and updates model
                if (options.is_slamming) {
                    auto start = std::chrono::high_resolution_clock::now();

                    // load new frame into octree collision object and compute overlap
                    octree.switchBuffers();
                    octree.setInputCloud(preview_cloud);
                    octree.addPointsFromInputCloud();
                    newPointIdxVector.clear();

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

                    // time benchmark stop
                    auto end = std::chrono::high_resolution_clock::now();
                    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                    frames_processed++;
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

                    // visualize world model
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(model, "x");
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(model, rgb, "model")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(model, rgb, "model");
                    }





// preview cloud visualization
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgbb(ground_cloud, 255,0 ,0);
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(ground_cloud, rgbb, "ground ground")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(ground_cloud, rgbb, "ground ground");
                    }





                    // preview cloud visualization
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbc(preview_cloud);
                    if (!viewer->updatePointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "sample ground")) {
                        viewer->addPointCloud<pcl::PointXYZRGB>(preview_cloud, rgbc, "sample ground");
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
    std::cout << "Frame count: " << frames_processed << " avg time of mapping: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
    std::cout << "Main thread exitting.." << std::endl;
    is_running = false;
}

void app::Application::stop() {
    is_running = false;
}