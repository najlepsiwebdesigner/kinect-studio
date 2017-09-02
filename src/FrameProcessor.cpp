//
// Created by Peter Beno on 02/09/2017.
//

#include "FrameProcessor.h"
#include "Application.h"

#include "parallel_threshold.h"
#include "fastBilateral.hpp"

#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


void generatePointCloud(const cv::Mat & rgbMat, const cv::Mat & depthMat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud) {
    const float focalLength = 525;
    const float centerX = 319.5;
    const float centerY = 239.5;
    const float scalingFactor = 1; //5000.0;
    int depth_idx = 0;

    for (int v = 0; v < 480; ++v)
    {
        for (int u = 0; u < 640; ++u, ++depth_idx)
        {
            pcl::PointXYZRGB & pt = pointcloud->points[depth_idx];
            pt.z = depthMat.at<uint16_t>(v,u) / scalingFactor;
            pt.x = (static_cast<float> (u) - centerX) * pt.z / focalLength;
            pt.y = (static_cast<float> (v) - centerY) * pt.z / focalLength;
            pt.b = rgbMat.at<cv::Vec3b>(v,u)[0];
            pt.g = rgbMat.at<cv::Vec3b>(v,u)[1];
            pt.r = rgbMat.at<cv::Vec3b>(v,u)[2];
        }
    }

}



void computeBearingAngleImage(const cv::Mat & depthMat, cv::Mat & output) {

    int depth_idx = 0;
    const double PI = 3.14159265358979323846;

#pragma  omp parallel for
    for (unsigned int v = 0; v < 480; ++v)
    {
        for (unsigned int u = 0; u < 640; ++u, ++depth_idx)
        {

            int point1_z = depthMat.at<uint16_t>(v,u);

            if (point1_z != 0) {

                int point1_x = u;
                int point1_y = v;


                int point2_x = u;
                int point2_y = v + 1;
                int point2_z = depthMat.at<uint16_t>(v + 1, u + 1);


                double a, b, c;
                double theta;
                a = sqrt(point1_x * point1_x + point1_y * point1_y + point1_z * point1_z);
                b = sqrt((point1_x - point2_x) * (point1_x - point2_x)
                         + (point1_y - point2_y) * (point1_y - point2_y)
                         + (point1_z - point2_z) * (point1_z - point2_z));
                c = sqrt(point2_x * point2_x + point2_y * point2_y + point2_z * point2_z);

                if (a != 0 && b != 0) {
                    theta = acos((a * a + b * b - c * c) / (2 * a * b)) * 180 / PI;
                } else {
                    theta = 0;
                }

                double gray;
                gray = theta / 180 * 255;


                output.at<uint16_t>(v, u) = gray;
            } else {
                output.at<uint16_t>(v, u) = 0;
            }


        }
    }



//    depthMat.copyTo(output);


}



namespace app {
    void FrameProcessor::run() {

        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        long long frame_processing_average_milliseconds = 0;
        long long frames_processed = 0;


        while (app::Application::is_running) {

            app::Frame temp_frame;


            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud_ptr->width = 640;
            cloud_ptr->height = 480;
            cloud_ptr->points.resize (640*480);


            bool process_frame = false;

            {
                std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);

                // mark this frame as visited and pass it to processing pipeline
                if (current_frame.unread) {
                    current_frame.unread = false;
                    process_frame = true;
                    temp_frame = current_frame;
                }
            }

            // processing pipeline start
            if (process_frame){

                // time measurement
                start = std::chrono::high_resolution_clock::now();

//////////////////////////// processing starts here
                {
                    // lock currently processed frame
                    std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);



                    // ### opencv fast parallel Thresholding - 0 ms
                    temp_frame.thresholdedDepthMat = temp_frame.depthMat.clone();
                    parallel_for_(cv::Range(0, temp_frame.thresholdedDepthMat.rows),
                                  Parallel_Threshold(temp_frame.thresholdedDepthMat, 400, 8000));



                    // ### generate point cloud - 11 ms
                    // generatePointCloud(temp_frame.rgbMat, temp_frame.depthMat, cloud_ptr);

                    temp_frame.baMat = temp_frame.thresholdedDepthMat.clone();
                    computeBearingAngleImage(temp_frame.baMat.clone(), temp_frame.baMat);

                    // ### bilateral filter is crazy slow
//                    cv_extend::bilateralFilter(temp_frame.baMat.clone(),
//                                               temp_frame.baMat, 100 ,10);

                    processed_frame = temp_frame;
                    processed_frame.processed = true;


                }
//////////////////////////// processing ends here

                // time measurement
                end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                frames_processed++;


            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

        }

        std::cout << "Frame count: " << frames_processed << " avg time of processing: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;

        std::cout << "Processing thread exitting.." << std::endl;
    }
}
