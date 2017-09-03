//
// Created by Peter Beno on 02/09/2017.
//

#include "Application.h"


void matchAndDraw(const cv::Mat & img_1, const cv::Mat & img_2, cv::Mat img_1_result = cv::Mat(), cv::Mat img_2_result = cv::Mat()) {



    using namespace cv;
    using namespace cv::xfeatures2d;

    if( !img_1.data || !img_2.data ){
        std::cout<< " --(!) Error reading images " << std::endl;
        return;
    }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

//    Ptr<SURF> detector = SURF::create( minHessian );
    // Ptr<AKAZE> detector = AKAZE::create();
//     Ptr<ORB> detector = ORB::create();
    // Ptr<SIFT> detector = SIFT::create();
    // Ptr<MSER> detector = MSER::create();
    // Ptr<BRISK> detector = BRISK::create();
    // Ptr<KAZE> detector = KAZE::create();
    // Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();

    Ptr<ORB> detector = ORB::create();

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector->detect( img_1, keypoints_1 );
    detector->detect( img_2, keypoints_2 );

    //-- Draw keypoints
    Mat img_keypoints_1; Mat img_keypoints_2;

    drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

    //-- Step 2: Calculate descriptors (feature vectors)
    Ptr<SURF> extractor = SURF::create();

    Mat descriptors_1, descriptors_2;

    extractor->compute( img_1, keypoints_1, descriptors_1 );
    extractor->compute( img_2, keypoints_2, descriptors_2 );


    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ ){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

     printf("-- Max dist : %f \n", max_dist );
     printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ ){
        if( matches[i].distance <= max(2*min_dist, 0.02) ){
            good_matches.push_back( matches[i]);
        }
    }





    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1_result, keypoints_1, img_2_result, keypoints_2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    drawKeypoints(img_1, keypoints_1, img_1_result, Scalar(255), DrawMatchesFlags::DRAW_OVER_OUTIMG );
    imshow("1", img_1_result);

    //-- Show detected matches
    imshow( "Good Matches", img_matches );




    // for( int i = 0; i < (int)good_matches.size(); i++ ){
    //     printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
    // }
//    waitKey(0);
}









bool app::Application::is_running = false;

void app::Application::start() {




    cv::namedWindow("Depth");
    cv::namedWindow("Video");
    cv::namedWindow("Thresh");
    cv::namedWindow("Good Matches");
    cv::namedWindow("Keypoints");


    // holds latest captured frame from data source
    Frame grabbed_frame;
    std::mutex grabbed_frame_mutex;

    // holds last processed frame from data source
    Frame processed_frame;
    std::mutex processed_frame_mutex;

    FrameGenerator frameGenerator(grabbed_frame, grabbed_frame_mutex);
    FrameProcessor frameProcessor(grabbed_frame, grabbed_frame_mutex, processed_frame, processed_frame_mutex);

    is_running = true;



    std::thread t1([&frameGenerator]() {
        frameGenerator.run();
    });

    std::thread t2([&frameProcessor]() {
        frameProcessor.run();
    });



    bool is_first = true;
    cv::Mat previous_processed_baf;


    while (is_running) {
        {
            std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);





            if (processed_frame.processed) {

                cv::Mat depthf(cv::Size(640, 480), CV_8UC1);
                processed_frame.depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);

//                cv::Mat threshf(cv::Size(640, 480), CV_8UC1);
//                processed_frame.thresholdedDepthMat.convertTo(threshf, CV_8UC1, 255.0 / 2048.0);

                cv::Mat baf(cv::Size(640, 480), CV_8UC1);
                processed_frame.baMat.convertTo(baf, CV_8UC1, 1);

                cv::imshow("Video", processed_frame.rgbMat);
                cv::imshow("Depth", depthf);

                cv::Mat img_keypoints;

                drawKeypoints( processed_frame.rgbMat, processed_frame.keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );



//                cv::Mat sobelf = baf.clone();
//                sobel(sobelf);
//                sobel(sobelf);

                cv::imshow("Keypoints", img_keypoints);

//                cv::imshow("BA",baf);

//                if (is_first) {
//                    is_first = false;
//                } else {
//                    matchAndDraw(previous_processed_baf,baf);
//                }
//



                previous_processed_baf = processed_frame.baMat.clone();
            }


//            std::lock_guard<std::mutex> mutex_guard(grabbed_frame_mutex);
//
//            cv::imshow("Depth", grabbed_frame.depthMat);
//            cv::imshow("Video", grabbed_frame.rgbMat);
        }

        if (cvWaitKey(10) >= 0) {
            is_running = false;
            break;
        }
    }


    t1.join();
    t2.join();

}

void app::Application::stop() {
    is_running = false;
}