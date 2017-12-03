//
// Created by Peter Beno on 02/09/2017.
//

#ifndef APP_FRAME_H
#define APP_FRAME_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace app {
    class Frame {
    public:
        cv::Mat rgbMat;
        cv::Mat depthMat;
        cv::Mat baMat;
        cv::Mat hsvMat;
        cv::Mat claheMat;

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals;

        long long order;

        bool generated;
        bool processed;
        bool matched;

        bool t1_done;
        bool t2_done;
        bool t3_done;
        bool t4_done;

        long double x;
        long double y;
        long double theta;

        Eigen::Affine3f transform_odometry;
        Eigen::Affine3f transform_visual;

        Frame() :
                rgbMat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0)),
                depthMat(cv::Size(640, 480), CV_16UC1),
                baMat(cv::Size(640,480), CV_8UC1),
                hsvMat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0)),
                claheMat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0)),
                generated(false),
                processed(false),
                matched(false),
                t1_done(false),
                t2_done(false),
                t3_done(false),
                t4_done(false),
                cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
        {

        }
    };
}
#endif //APP_FRAME_H
