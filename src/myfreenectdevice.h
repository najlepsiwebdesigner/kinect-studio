#ifndef MYFREENECTDEVICE_H
#define MYFREENECTDEVICE_H
#include <pthread.h>
#include <vector>
#include "libfreenect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// helper class for MyFreenectDevice
class myMutex {
    public:
        myMutex() {
            pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
            pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
            pthread_mutex_unlock( &m_mutex );
        }
    private:
        pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
    public:
        MyFreenectDevice(freenect_context *_ctx, int _index);

        // Do not call directly even in child
        void VideoCallback(void* _rgb, uint32_t timestamp);
        // Do not call directly even in child
        void DepthCallback(void* _depth, uint32_t timestamp);
        bool getVideo(cv::Mat& output);
        bool getDepth(cv::Mat& output);
    private:
        std::vector<uint8_t> m_buffer_depth;
        std::vector<uint8_t> m_buffer_rgb;
        std::vector<uint16_t> m_gamma;
        cv::Mat depthMat;
        cv::Mat rgbMat;
        cv::Mat ownMat;
        myMutex m_rgb_mutex;
        myMutex m_depth_mutex;
        bool m_new_rgb_frame;
        bool m_new_depth_frame;
};
#endif // MYFREENECTDEVICE_H
