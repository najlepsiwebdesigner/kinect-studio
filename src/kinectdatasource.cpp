#include "kinectdatasource.h"
#include <iostream>

bool KinectDataSource::getDepth(cv::Mat& output) {
    depth_iteration++;
    if (depth_iteration % everyNth == 0) {
        device->getDepth(output);
    } else {
        cv::Mat tmp;
        device->getDepth(tmp);
    }

    return true;
}


bool KinectDataSource::getVideo(cv::Mat& output) {
    rgb_iteration++;
    if (rgb_iteration % everyNth == 0) {
        device->getVideo(output);
    } else {
        cv::Mat tmp;
        device->getVideo(tmp);
    }
    return true;
}


bool KinectDataSource::getVideoAndDepth(cv::Mat& video, cv::Mat& depth) {
    rgb_iteration++;
    depth_iteration++;
    device->getVideo(video);
    device->getDepth(depth);

    return true;
}



KinectDataSource::KinectDataSource() { 
    freenect_ptr = new Freenect::Freenect;
    device = &freenect_ptr->createDevice<MyFreenectDevice>(0);

    device->startVideo();
    device->startDepth();
}


KinectDataSource::~KinectDataSource() {
    device->stopVideo();
    device->stopDepth();

    delete this->device;
    this->device = NULL;

    delete this->freenect_ptr;
    this->freenect_ptr = NULL;
}
