#ifndef RGBDDATASOURCE_H
#define RGBDDATASOURCE_H

#include <opencv2/core/core.hpp>

// interface class
class RgbdDataSource {
    public:
        virtual bool getDepth(cv::Mat& output)=0;
        virtual bool getVideo(cv::Mat& output)=0;
        bool recording_enabled = false;
};

#endif  // RGBDDATASOURCE_H
