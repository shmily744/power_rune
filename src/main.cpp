#include <opencv2/opencv.hpp>

#include "RuneDetector.h"

int main() {
    cv::VideoCapture cap(R"(../video/blue.mp4)");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
    }

    rm_power_rune::RuneDetector runeDetector(180,100,rm_power_rune::BLUE);

    while (cap.isOpened())
    {
        // Initialise frame matrix
        cv::Mat rawImages;

        // Initialize a boolean to check if frames are there or not
        bool isSuccess = cap.read(rawImages);

        // If frames are present, show it
        if(isSuccess)
        {
            //display frames
            runeDetector.detect(rawImages);
            int key = runeDetector.drawResults(rawImages);
            if(key == 'q') break;
        }else
        {
            std::cout << "Video camera is disconnected" << std::endl;
            break;
        }
    }
    // Release the video capture object
    cap.release();
    cv::destroyAllWindows();
    return 0;

}
