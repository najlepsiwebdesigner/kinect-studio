#ifndef GLOBALS_H
#define GLOBALS_H

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <boost/filesystem.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "Bench.h"

#include "Frame.h"


void sobel(cv::Mat & src_gray);

struct RobotPose {
    long double x;
    long double y;
    long double theta;
    long double dx;
    long double dy;
    long double dtheta;
};

struct Options {
    bool show_3D = true;
    bool show2D = false;
    bool is_recording = false;
    bool offline = false;
    bool is_slamming = true;
    std::string record_path;
};




#endif  // GLOBALS_H
