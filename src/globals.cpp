//
// Created by Peter Beno on 02/09/2017.
//
#include "globals.h"

using namespace cv;
using namespace std;

/**
 * @brief      converts cv::Mat.type() output to readable form, source: http://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
 *
 * @param[in]  type  The type
 *
 * @return     readable form of Mat.type()
 */
string cvType2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}






/**
 * @brief      macro for printing out error messages via cout
 *
 * @param      X     message to be printed
 *
 * @return     void
 */
#define DEBUG(X) \
 ( \
  (std::cout << (X) << std::endl), \
  (void)0 \
 )


/**
 * @brief      if directory is not wxisting, create it
 *
 * @param[in]  dirname  The dirname
 *
 * @return     returns 1 when directory was successfully created,
 *             0 when it already exists and -1 when it could not be created
 */
int make_dir(string dirname) {
    boost::filesystem::path dir(dirname.c_str());
    if(!(boost::filesystem::exists(dir))) {

        if (boost::filesystem::create_directory(dir)) {
            return 1;
        }
    } else {
        return 0;
    }
    return -1;
}



void resizedownup(Mat & image){
    Mat small;
    cv::resize(image, small,Size(320,240),0,0);
    cv::medianBlur(small, small, 9);
    cv::resize(small, image, Size(image.cols,image.rows));
}


// void kMeans() {
//     const int MAX_CLUSTERS = 5;
//     Scalar colorTab[] =
//     {
//         Scalar(0, 0, 255),
//         Scalar(0,255,0),
//         Scalar(255,100,100),
//         Scalar(255,0,255),
//         Scalar(0,255,255)
//     };
//     Mat img(500, 500, CV_8UC3);
//     RNG rng(12345);
//     for(;;)
//     {
//         int k, clusterCount = rng.uniform(2, MAX_CLUSTERS+1);
//         int i, sampleCount = rng.uniform(1, 1001);
//         Mat points(sampleCount, 1, CV_32FC2), labels;
//         clusterCount = MIN(clusterCount, sampleCount);
//         Mat centers;
//         /* generate random sample from multigaussian distribution */
//         for( k = 0; k < clusterCount; k++ )
//         {
//             Point center;
//             center.x = rng.uniform(0, img.cols);
//             center.y = rng.uniform(0, img.rows);
//             Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
//                                              k == clusterCount - 1 ? sampleCount :
//                                              (k+1)*sampleCount/clusterCount);
//             rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
//         }
//         randShuffle(points, 1, &rng);
//         kmeans(points, clusterCount, labels,
//             TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
//                3, KMEANS_PP_CENTERS, centers);
//         img = Scalar::all(0);
//         for( i = 0; i < sampleCount; i++ )
//         {
//             int clusterIdx = labels.at<int>(i);
//             Point ipt = points.at<Point2f>(i);
//             circle( img, ipt, 2, colorTab[clusterIdx], FILLED, LINE_AA );
//         }
//         imshow("clusters", img);
//         char key = (char)waitKey();
//         if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
//             break;
//     }
// }



// void kMeans(Mat & src) {
//     cv::Mat samples(src.total(), 3, CV_32F);
//        auto samples_ptr = samples.ptr<float>(0);
//        for( int row = 0; row != src.rows; ++row){
//            auto src_begin = src.ptr<uchar>(row);
//            auto src_end = src_begin + src.cols * src.channels();
//            //auto samples_ptr = samples.ptr<float>(row * src.cols);
//            while(src_begin != src_end){
//                samples_ptr[0] = src_begin[0];
//                samples_ptr[1] = src_begin[1];
//                samples_ptr[2] = src_begin[2];
//                samples_ptr += 3; src_begin +=3;
//            }
//        }

//        std::cout << "vysamploval som v kmeans" << std::endl;


//        //step 2 : apply kmeans to find labels and centers
//        int clusterCount = 3;
//        cv::Mat labels;
//        int attempts = 5;
//        cv::Mat centers;
//        cv::kmeans(samples, clusterCount, labels,
//                   cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
//                                    10, 0.01),
//                   attempts, cv::KMEANS_PP_CENTERS, centers);

//        //step 3 : map the centers to the output
//        cv::Mat new_image(src.size(), src.type());
//        for( int row = 0; row != src.rows; ++row){
//            auto new_image_begin = new_image.ptr<uchar>(row);
//            auto new_image_end = new_image_begin + new_image.cols * 3;
//            auto labels_ptr = labels.ptr<int>(row * src.cols);

//            while(new_image_begin != new_image_end){
//                int const cluster_idx = *labels_ptr;
//                auto centers_ptr = centers.ptr<float>(cluster_idx);
//                new_image_begin[0] = centers_ptr[0];
//                new_image_begin[1] = centers_ptr[1];
//                new_image_begin[2] = centers_ptr[2];
//                new_image_begin += 3; ++labels_ptr;
//            }
//        }

//        src = new_image.clone();
// }




void erosion(Mat & image) {
    //get the structuring of rectangle, size is 5 * 5
    cv::Mat const shape = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(image, image, shape);
}


void dilation(Mat & image) {
    dilate(image, image, Mat(), Point(-1, -1), 2, 1, 1);
}




void sobel(Mat & src_gray) {
    GaussianBlur( src_gray, src_gray, Size(3,3), 0, 0, BORDER_DEFAULT );
    Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    // /// Convert it to gray
    // cvtColor( src, src_gray, COLOR_RGB2GRAY );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );


//    cv::dilate(grad, grad, Mat(), Point(1,-1));
    // grad.copyTo(src_gray);


    imshow("Sobel", grad);

    // cvtColor( grad, src_gray, COLOR_GRAY2RGB );
}

