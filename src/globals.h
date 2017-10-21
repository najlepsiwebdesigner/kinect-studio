#ifndef UTILS_H
#define UTILS_H

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


#include "Frame.h"


void sobel(cv::Mat & src_gray);

struct RobotPose {
    long double x;
    long double y;
    long double theta;
};





#endif  // UTILS_H
