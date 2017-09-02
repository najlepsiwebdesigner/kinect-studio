#ifndef OFFLINEDATASOURCE_H
#define OFFLINEDATASOURCE_H

#include <vector>
#include <string.h>
#include <boost/filesystem.hpp>
#include "rgbddatasource.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace boost::filesystem;

// OfflineDataSource
class OfflineDataSource : public RgbdDataSource {
    public:
        OfflineDataSource();
        ~OfflineDataSource();
        bool getDepth(cv::Mat& output);
        bool getVideo(cv::Mat& output);
        bool getVideoAndDepth(cv::Mat& video, cv::Mat& depth);
        std::vector<string> rgb_filenames;
        std::vector<string> depth_filenames;
        long rgb_iteration = 0;
        long depth_iteration = 0;
        long iteration = 0;
        long frequency = 30; // [Hz] unused!!!!
        bool recording_enabled = false;
        bool loop = true;
};


#endif // OFFLINEDATASOURCE_H
