//
// Created by Peter Beno on 02/09/2017.
//

#include "FrameProcessor.h"
#include "Application.h"


#include "parallel_threshold.h"


namespace app {
    void FrameProcessor::run() {


        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        long long frame_processing_average_milliseconds = 0;
        long long frames_processed = 0;


        while (app::Application::is_running) {

            app::Frame temp_frame;
            bool process_frame = false;

            {
                std::lock_guard<std::mutex> mutex_guard(current_frame_mutex);

                // mark this frame as visited and pass it to processing pipeline
                if (current_frame.unread) {
                    current_frame.unread = false;
                    process_frame = true;
                    temp_frame = current_frame;
                }
            }

            // processing pipeline start
            if (process_frame){

                // time measurement
                start = std::chrono::high_resolution_clock::now();

//////////////////////////// processing starts here
                // lock currently processed frame
                std::lock_guard<std::mutex> mutex_guard(processed_frame_mutex);

                // optional delay
                 usleep(10 * 1000);

                // opencv fast parallel Thresholding
                temp_frame.thresholdedDepthMat = temp_frame.depthMat.clone();
                parallel_for_( cv::Range(0,temp_frame.thresholdedDepthMat.rows) , Parallel_Threshold(temp_frame.thresholdedDepthMat, 400, 6000)) ;





                processed_frame = temp_frame;
                processed_frame.processed = true;

//////////////////////////// processing ends here

                // time measurement
                end = std::chrono::high_resolution_clock::now();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                frame_processing_average_milliseconds = (frame_processing_average_milliseconds * frames_processed + millis) / (frames_processed + 1);
                frames_processed++;

                std::cout << "Frame number: " << frames_processed << " avg time: " << frame_processing_average_milliseconds << " [ms]"<< std::endl;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

        }

        std::cout << "Processing thread exitting.." << std::endl;
    }
}
