#ifndef CV_PARALLEL_THRESHOLD
#define CV_PARALLEL_THRESHOLD

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @brief      Parallel threshold implementation with opencv, source http://answers.opencv.org/question/22115/best-way-to-apply-a-function-to-each-element-of-mat/
 *
 * @param[in]   max Maximum depth value to be kept. Values higher will be set to zero
 * @param[in]   min Nimimum depth value to be kept. Values lower will be set to zero
 */
class Parallel_Threshold: public cv::ParallelLoopBody {   
public:
    int max_depth = 10000;
    int min_depth = 400;

    Parallel_Threshold(cv::Mat &imgg, int min, int max) : img(imgg), min_depth(min), max_depth(max)
    {}

    void operator() (const cv::Range &r) const
    {
        for(int j=r.start; j<r.end; ++j)
        {
            uint16_t *ptr = const_cast<uint16_t*>(img.ptr<uint16_t>(j));
            int width = img.cols;
            for (int x = 0; x<width; x++,ptr++)
            {
                if (*ptr > max_depth or *ptr < min_depth) {
                    *ptr = 0;
                }
            }
        }
    }

private:
    cv::Mat img;
};





#endif  // CV_PARALLEL_THRESHOLD