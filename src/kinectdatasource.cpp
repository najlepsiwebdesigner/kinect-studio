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


bool KinectDataSource::getVideoAndDepth(cv::Mat& video, cv::Mat& depth, bool & new_frame_arrived) {
    
    bool video_ok = device->getVideo(video);
    bool depth_ok = device->getDepth(depth);

    rgb_iteration++;
    depth_iteration++;

    if (video_ok && depth_ok) {
        new_frame_arrived = true;
        // std::cout << "new frame!" << std::endl; 
    } 
    else {
        new_frame_arrived = false;
    }

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

    freenect_ptr->deleteDevice(0);
//    delete this->device;
//    this->device = NULL;

    delete this->freenect_ptr;
    this->freenect_ptr = NULL;
}
