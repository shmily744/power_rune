#include <opencv2/opencv.hpp>

#include "RuneDetector.h"

int main() {
    cv::VideoCapture cap(R"(../video/blue.mp4)");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
    }

    rm_power_rune::RuneDetector runeDetector(180, 100, rm_power_rune::BLUE, 180);

    while (cap.isOpened()) {
        // Initialise frame matrix
        cv::Mat rawImages;

        // Initialize a boolean to check if frames are there or not
        bool isSuccess = cap.read(rawImages);

        // If frames are present, show it
        if (isSuccess) {
            // 开始计时
            auto startTime = std::chrono::steady_clock::now();

            //display frames
            runeDetector.detect(rawImages);
            int key = runeDetector.drawResults(rawImages);
            if (key == 'q') break;

            // 计算总帧时间
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedTime = std::chrono::duration<double>(endTime - startTime).count();

            // 如果时间间隔达到了设定的秒数，计算并输出帧率
            double fps = 1 / elapsedTime;
            std::cout << "FPS: " << fps << std::endl;
        } else {
            std::cout << "Video camera is disconnected" << std::endl;
            break;
        }
    }
    // Release the video capture object
    cap.release();
    cv::destroyAllWindows();
    return 0;
}