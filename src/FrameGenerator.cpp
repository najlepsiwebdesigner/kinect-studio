//
// Created by Peter Beno on 02/09/2017.
//

#include "FrameGenerator.h"
#include "offlinedatasource.h"
#include "kinectdatasource.h"
#include "Application.h"

// using namespace std::chrono_literals;
using namespace std::chrono;

namespace app {
    // using namespace std::chrono_literals;
    
    void FrameGenerator::run() {

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(1);

        string rgb_path("/Volumes/rdisk/rgb/");
        string depth_path("/Volumes/rdisk/depth/");

        string suffix(".png");
        string rgb_filename("");
        string depth_filename("");


        cv::Mat video;
        cv::Mat depth;
        int i = 0;

        long double prevX = 0;
        long double prevY = 0;
        long double prevTheta = 0;


        if (!options.offline) {
            // while we have frames
            std::ofstream odometry_file("/Volumes/rdisk/odometry.txt");
            KinectDataSource offlineDataSource;
            while (offlineDataSource.getVideoAndDepth(video, depth)) {

                if (!app::Application::is_running) break;

                if (depth.cols == 0) {
                    continue;
                }

                // create frame
                Frame frame;

                frame.rgbMat = video.clone();
                frame.depthMat = depth.clone();

                frame.order = i;
                frame.generated = true;
                frame.x = current_robot_pose.x;
                frame.y = current_robot_pose.y;
                frame.theta = current_robot_pose.theta;


                std::cout << "X: " <<  frame.x << " Y:" << frame.y << " theta: " << frame.theta << std::endl;

                // Compute odometry transformation
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << frame.y*1000, 0.0,frame.x*1000;
                transform.rotate (Eigen::AngleAxisf (frame.theta, Eigen::Vector3f::UnitY()));
                frame.transform_odometry = transform;


                if (options.is_recording) {
                    milliseconds ms = duration_cast<milliseconds>(
                            system_clock::now().time_since_epoch()
                    );
                    std::ostringstream mlss;
                    mlss << ms.count();
                    std::string milli_str = mlss.str();

                    std::ostringstream rgb_file;
                    std::ostringstream depth_file;

                    rgb_file << rgb_path << rgb_filename << milli_str << suffix;
                    depth_file << depth_path << depth_filename << milli_str << suffix;

                    cv::imwrite(rgb_file.str(), frame.rgbMat, compression_params);
                    cv::imwrite(depth_file.str(), frame.depthMat, compression_params);
                    odometry_file << frame.x << " " << frame.y << " " << frame.theta << std::endl;

                    // std::cout << "frame name: " <<  rgb_file.str() << std::endl;
                }


                // thread safe write to shared variable
                //if (grabbed_frame.t2_done || i == 0) 
                {
                    std::lock_guard<std::mutex> mutex_guard(grabbed_frame_mutex);
                    grabbed_frame = frame;
                    grabbed_frame.t1_done = true;
                }


                i++;
            }

            app::Application::stop();

            std::cout << "Generator thread exitting.." << std::endl;
        }
        else {

            std::ifstream odometry_file("/Volumes/rdisk/odometry.txt");
            OfflineDataSource offlineDataSource;

            while (offlineDataSource.getVideoAndDepth(video, depth)) {

                if (!app::Application::is_running) break;

                if (depth.cols == 0) {
                    continue;
                }

                // create frame
                Frame frame;

                frame.rgbMat = video.clone();
                frame.depthMat = depth.clone();

                frame.order = i;
                frame.generated = true;

                long double x, y , theta;
                odometry_file >> frame.x >> frame.y >> frame.theta;

                // Compute odometry transformation
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                transform.translation() << frame.y*1000, 0.0,frame.x*1000;
                transform.rotate (Eigen::AngleAxisf (frame.theta, Eigen::Vector3f::UnitY()));
                frame.transform_odometry = transform;

                


                // thread safe write to shared variable
                if (grabbed_frame.t2_done || i == 0) {
                    std::lock_guard<std::mutex> mutex_guard(grabbed_frame_mutex);
                    grabbed_frame = frame;
                    grabbed_frame.t1_done = true;
                }

                i++;

                // simulate som frame rate
                 std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            app::Application::stop();

            std::cout << "Generator thread exitting.." << std::endl;
        }
    }

}