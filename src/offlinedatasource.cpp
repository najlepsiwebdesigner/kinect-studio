#include "offlinedatasource.h"
#include "strnatcmp.h"

using namespace cv;
using namespace std;


bool OfflineDataSource::getVideoAndDepth(cv::Mat& video, cv::Mat& depth) {
    if ((iteration >= rgb_filenames.size() || iteration >= depth_filenames.size()) && loop) {
        iteration = 0;
    }

    cv::Mat videoImage = imread(rgb_filenames[iteration], CV_LOAD_IMAGE_COLOR);
    cv::Mat depthImage = imread(depth_filenames[iteration], cv::IMREAD_UNCHANGED);

//    std::cout << rgb_filenames[iteration] << std::endl;
//    std::cout << depth_filenames[iteration] << std::endl;


//    if ((videoImage.size().height == 0 || videoImage.size().width == 0)) return false;
//    if ((depthImage.size().height == 0 || depthImage.size().width == 0)) return false;

    iteration++;
    videoImage.copyTo(video);
    depthImage.copyTo(depth);

    return true;
}


bool OfflineDataSource::getDepth(cv::Mat& output) {
    if (! (depth_iteration < depth_filenames.size())) {
        if (loop) {
            depth_iteration = 0;
        } else {
            return false;
        }
    }
    cv::Mat image = imread(depth_filenames[depth_iteration], cv::IMREAD_UNCHANGED);

    if ((image.size().height == 0 || image.size().width == 0)) return false;

    depth_iteration++;
    image.copyTo(output);
    usleep(1000000/frequency);
    return true;
}


bool OfflineDataSource::getVideo(cv::Mat& output) {
    if (! (rgb_iteration < rgb_filenames.size())) {
        if (loop) {
            rgb_iteration = 0;
        } else {
            return false;
        }
    }
    cv::Mat image = imread(rgb_filenames[rgb_iteration], CV_LOAD_IMAGE_COLOR);
    
    if ((image.size().height == 0 || image.size().width == 0)) return false;

    rgb_iteration++;
    image.copyTo(output);
    usleep(1000000/frequency);
    return true;
}

OfflineDataSource::OfflineDataSource() {
    // load whole directory here and sort the contents by name
    boost::filesystem::path rgb_directory ("/Volumes/rdisk//rgb");
    boost::filesystem::path depth_directory ("/Volumes/rdisk//depth");

    if (!exists(rgb_directory) || !is_directory(rgb_directory)) {
        std::cout << "RGB input directory does not exist or is not a directory!" << std::endl;
        exit(0);
    }

    if (!exists(depth_directory) || !is_directory(depth_directory)) {
        std::cout << "Depth input directory does not exist or is not a directory!" << std::endl;
        exit(0);
    }

    std::cout << "Wait please..." << std::endl << std::endl;


    directory_iterator rgb_end_itr;
    for (directory_iterator itr(rgb_directory); itr != rgb_end_itr; ++itr) {
        if (is_regular_file(itr->path())) {
            std::string current_file = itr->path().string();
            rgb_filenames.push_back(current_file);
        }
    }

    std::cout << "Number of rgb frames:" << rgb_filenames.size() << std::endl;

    std::sort(rgb_filenames.begin(), rgb_filenames.end(),compareNat);

    directory_iterator depth_end_itr;
    for (directory_iterator itr(depth_directory); itr != depth_end_itr; ++itr) {
        if (is_regular_file(itr->path())) {
            std::string current_file = itr->path().string();
            depth_filenames.push_back(current_file);
        }
    }

    std::cout << "Number of depth frames:" << depth_filenames.size() << std::endl;

    std::sort(depth_filenames.begin(), depth_filenames.end(),compareNat);

    if (depth_filenames.size() < 1 || rgb_filenames.size() < 1) {
        throw std::invalid_argument("No files for offline processing!");
    }
}


OfflineDataSource::~OfflineDataSource() {
    // close the directory

}
