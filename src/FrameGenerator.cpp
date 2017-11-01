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
                frame.unread = true;
                frame.x = current_robot_pose.x;
                frame.y = current_robot_pose.y;
                frame.theta = current_robot_pose.theta;


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

//                std::cout << "frame name: " <<  rgb_file.str() << std::endl;
                }

                i++;

                // thread safe write to shared variable
                {
                    std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);
                    current_frame = frame;
                }
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
                frame.unread = true;

                long double x, y , theta;
                odometry_file >> frame.x >> frame.y >> frame.theta;

                i++;

                // thread safe write to shared variable
                {
                    std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);
                    current_frame = frame;
                }

                // simulate som frame rate
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            app::Application::stop();

            std::cout << "Generator thread exitting.." << std::endl;
        }
    }

}