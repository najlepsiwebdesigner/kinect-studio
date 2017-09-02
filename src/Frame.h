//
// Created by Peter Beno on 02/09/2017.
//

#ifndef APP_FRAME_H
#define APP_FRAME_H

namespace app {
    class Frame {
    public:
        cv::Mat rgbMat;
        cv::Mat depthMat;
        cv::Mat thresholdedDepthMat;
        long long order;
        bool unread;
        bool processed;

        Frame() :
                rgbMat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0)),
                depthMat(cv::Size(640, 480), CV_16UC1),
                thresholdedDepthMat(cv::Size(640,480), CV_8UC1),
                unread(true)
        {

        }
    };
}
#endif //APP_FRAME_H
