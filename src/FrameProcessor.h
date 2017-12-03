//
// Created by Peter Beno on 02/09/2017.
//


#ifndef APP_FRAMEPROCESSOR_H
#define APP_FRAMEPROCESSOR_H

#include "globals.h"

namespace app {

    class FrameProcessor {
    public:

        std::mutex & grabbed_frame_mutex;
        app::Frame & grabbed_frame;
        std::mutex & processed_frame_mutex;
        app::Frame & processed_frame;
        Options options;

        FrameProcessor(Options options,
                       app::Frame & grabbed_frame,
                       std::mutex & grabbed_frame_mutex,
                       app::Frame & processed_frame,
                       std::mutex & processed_frame_mutex):
            options(options),
            grabbed_frame(grabbed_frame),
            grabbed_frame_mutex(grabbed_frame_mutex),
            processed_frame(processed_frame),
            processed_frame_mutex(processed_frame_mutex)
        {
        };

        void thresholdDepth(Frame frame);
        void computeHsv(Frame & temp_frame);
        void bilateralDepth(Frame & temp_frame);
        void bilateralRgb(Frame & temp_frame);
        void guidedFilterDepth(Frame & temp_frame);
        void computeClahe(Frame & temp_frame);
        void computePointCloud(Frame & temp_frame);
        void computePointCloudWithNormals(Frame & temp_frame);
        void computeBearingAngleImage(Frame & temp_frame);
        void computeKeypoints(Frame & temp_frame);
        void computeDescriptors(Frame & temp_frame);

        void run();
    };

}
#endif //APP_FRAMEPROCESSOR_H
