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

#include <math.h>

#include "guidedfilter.h"















namespace app {


    void FrameProcessor::computeKeypoints(Frame & temp_frame) {
        using namespace cv;
        using namespace cv::xfeatures2d;

        /// ### feature detection ± 13 ms
//         Ptr<SURF> detector = SURF::create( 100,4,1,false,false );
//         Ptr<AKAZE> detector = AKAZE::create();
        Ptr<ORB> detector = ORB::create();
//         Ptr<SIFT> detector = SIFT::create();
        // Ptr<MSER> detector = MSER::create();
//         Ptr<BRISK> detector = BRISK::create();
//         Ptr<KAZE> detector = KAZE::create();
//         Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();

        std::vector<KeyPoint> keypoints;
        detector->detect( temp_frame.claheMat, keypoints);


        // keep only 3D keypoints
        std::vector<cv::KeyPoint> filtered_frame_keypoints;

        for (int i = 0; i < keypoints.size();i++) {
            if (temp_frame.depthMat.at<uint16_t>(keypoints[i].pt) != 0) {
                filtered_frame_keypoints.push_back(keypoints[i]);
            }
        }

        temp_frame.keypoints = filtered_frame_keypoints;
    }

    void FrameProcessor::computeDescriptors(Frame & temp_frame) {
        using namespace cv;
        using namespace cv::xfeatures2d;

        if (temp_frame.keypoints.size() < 1) {
            computeKeypoints(temp_frame);
        }

        auto & keypoints = temp_frame.keypoints;

        /// ### feature description
        Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
//        Ptr<SURF> extractor = SURF::create();
        Mat descriptors;

        extractor->compute(temp_frame.rgbMat, keypoints, temp_frame.descriptors);

    }






    /// ### generate BA image
    void FrameProcessor::computeBearingAngleImage(Frame & temp_frame) {
        auto getBearingAngle = [](pcl::PointXYZRGB point1, pcl::PointXYZRGB point2) -> double {
            const double PI = 3.14159265358979323846;
            double a, b, c;
            double theta;
            a = sqrt (point1.x * point1.x + point1.y * point1.y + point1.z * point1.z);
            b = sqrt ((point1.x - point2.x) * (point1.x - point2.x) +
                      (point1.y - point2.y) * (point1.y - point2.y) +
                      (point1.z - point2.z) * (point1.z - point2.z));
            c = sqrt (point2.x * point2.x + point2.y * point2.y + point2.z * point2.z);

            if (a != 0 && b != 0)
            {
                theta = acos ((a * a + b * b - c * c) / (2 * a * b)) * 180 / PI;
            }
            else
            {
                theta = 0;
            }
            return theta;
        };




        if (temp_frame.cloud->size() < 1) {
            computePointCloud(temp_frame);
        }

        auto pointcloud = temp_frame.cloud;

        int width = pointcloud->width;
        int height = pointcloud->height;
        unsigned int size = width * height;
        double theta, theta2;
        uint8_t gray;

        // primary transformation process
        for (int i = 0; i < height - 1; ++i) {
            for (int j = 0; j < width - 1; ++j)
            {
                theta = getBearingAngle(pointcloud->points[i * width + j + 1], pointcloud->points[(i + 1) * width + j]);
                theta2 = getBearingAngle(pointcloud->points[i * width + j + 2], pointcloud->points[(i + 2) * width + j]);
//            theta2 = theta;

                // based on the theta, calculate the gray value of every pixel point
                temp_frame.baMat.at<uint8_t>(i + 1, j) = (uint8_t) ((theta + theta2)/2) * 255 / 180;
            }
        }
    }


    /// ### compute pointcloud
    void FrameProcessor::computePointCloud(Frame & temp_frame) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_ptr->width = 640;
        cloud_ptr->height = 480;
        cloud_ptr->points.resize (640*480);

        const float focalLength = 525;
        const float centerX = 319.5;
        const float centerY = 239.5;
        const float scalingFactor = 1; //5000.0;
        int depth_idx = 0;

        for (int v = 0; v < 480; ++v)
        {
            for (int u = 0; u < 640; ++u, ++depth_idx)
            {
                pcl::PointXYZRGB & pt = cloud_ptr->points[depth_idx];

                pt.z = temp_frame.depthMat.at<uint16_t>(v,u) / scalingFactor;
                pt.x = (static_cast<float> (u) - centerX) * pt.z / focalLength;
                pt.y = (static_cast<float> (v) - centerY) * pt.z / focalLength;
                pt.b = temp_frame.rgbMat.at<cv::Vec3b>(v,u)[0];
                pt.g = temp_frame.rgbMat.at<cv::Vec3b>(v,u)[1];
                pt.r = temp_frame.rgbMat.at<cv::Vec3b>(v,u)[2];
            }
        }

        temp_frame.cloud = cloud_ptr;
    }



    void FrameProcessor::thresholdDepth(Frame frame){
        /// ### opencv fast parallel Thresholding - 0 ms
        frame.depthMat = frame.depthMat.clone();
        parallel_for_(cv::Range(0, frame.depthMat.rows),
                      Parallel_Threshold(frame.depthMat, 400, 8000));


    }


    void FrameProcessor::computeHsv(Frame & temp_frame) {
        /// ### HSV MODEL
        cv::cvtColor(temp_frame.rgbMat, temp_frame.hsvMat, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsv_planes;
        cv::split(temp_frame.hsvMat, hsv_planes);
        cv::Mat h = hsv_planes[0]; // H channel
        cv::Mat s = hsv_planes[1]; // S channel
        cv::Mat v = hsv_planes[2]; // V channel

        temp_frame.hsvMat = v;

    }

    /// ### bilateral filter is crazy slow ± 50 ms
    void FrameProcessor::bilateralDepth(Frame & temp_frame) {
        cv_extend::bilateralFilter(temp_frame.depthMat.clone(),
                                   temp_frame.depthMat, 100 ,10);
    }

    /// ### bilateral filter on rgb image
    void FrameProcessor::bilateralRgb(Frame & temp_frame) {
        cv::Mat bil = temp_frame.rgbMat.clone();
        cv::bilateralFilter(temp_frame.rgbMat,bil,5,30,30);
        temp_frame.rgbMat = bil;

    }

    /// ### guided filter https://github.com/atilimcetin/guided-filter
    void FrameProcessor::guidedFilterDepth(Frame & temp_frame) {
        cv::Mat I = temp_frame.depthMat;
        cv::Mat p = I;

        int r = 2; // try r=2, 4, or 8
        double eps = 0.2 * 0.2; // try eps=0.1^2, 0.2^2, 0.4^2

        eps *= 255 * 255;   // Because the intensity range of our images is [0, 255]

        cv::Mat q = guidedFilter(I, p, r, eps);
        temp_frame.depthMat = q;
    }

    /// ### CLAHE - 5 ms https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c/24341809#24341809
    void FrameProcessor::computeClahe(Frame & temp_frame) {
        // READ RGB color image and convert it to Lab
        cv::Mat bgr_image = temp_frame.rgbMat;
        cv::Mat lab_image;
        cv::cvtColor(temp_frame.rgbMat, lab_image, CV_BGR2Lab);

        // Extract the L channel
        std::vector<cv::Mat> lab_planes(3);
        cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

        // apply the CLAHE algorithm to the L channel
        cv::Mat dst;
        // CLAHE instantiation
        static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(lab_planes[0], dst);

        // Merge the the color planes back into an Lab image
        dst.copyTo(lab_planes[0]);
        cv::merge(lab_planes, lab_image);

        // convert back to RGB
        cv::Mat image_clahe;
        cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

        temp_frame.claheMat = image_clahe;
    }



    void FrameProcessor::run() {
        /// start measuring for benchmarking
        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        long long frame_processing_average_milliseconds = 0;
        long long frames_processed = 0;

        while (app::Application::is_running) {
            app::Frame temp_frame;
            bool process_frame = false;

            // check if current_frame has changed, if yes, keep it and mark it for processing
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
                start = std::chrono::high_resolution_clock::now();

//////////////////////////// processing starts here

//                thresholdDepth(temp_frame);

                /// ### generate point cloud ± 11 ms
                computePointCloud(temp_frame);

//                computeBearingAngleImage(temp_frame);

                /// ### 5 ms
                computeClahe(temp_frame);

                /// ### compute keypoints and descriptors
                computeKeypoints(temp_frame);
                computeDescriptors(temp_frame);









//////////////////////////// processing ends here
                {
                    // lock currently processed frame
                    std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);
                    processed_frame = temp_frame;
                    processed_frame.processed = true;

                }

                end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                frames_processed++;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        /// exit thread
        std::cout << "Frame count: " << frames_processed << " avg time of processing: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
        std::cout << "Processing thread exitting.." << std::endl;
    }
}
