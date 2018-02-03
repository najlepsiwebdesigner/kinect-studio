#ifndef KINECTDATASOURCE_H
#define KINECTDATASOURCE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rgbddatasource.h"
#include "libfreenect.hpp"
#include "myfreenectdevice.h"


// KinectDataSource
class KinectDataSource : public RgbdDataSource {
    public:
        KinectDataSource();
        ~KinectDataSource();
        bool getDepth(cv::Mat& output);
        bool getVideo(cv::Mat& output);
        bool getVideoAndDepth(cv::Mat& video, cv::Mat& depth, bool & new_frame_arrived);
        Freenect::Freenect * freenect_ptr;
        MyFreenectDevice * device;
        bool recording_enabled = true;
        long everyNth = 1; // [take every nth frame]
        long rgb_iteration = 0;
        long depth_iteration = 0;
};
#endif // KINECTDATASOURCE_H