#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap(R"(../video/blue.mp4)");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
    }



    while (cap.isOpened())
    {
        // Initialise frame matrix
        cv::Mat frame;

        // Initialize a boolean to check if frames are there or not
        bool isSuccess = cap.read(frame);

        // If frames are present, show it
        if(isSuccess == true)
        {
            //display frames
            imshow("Frame", frame);
        }

        // If frames are not there, close it
        if (isSuccess == false)
        {
            std::cout << "Video camera is disconnected" << std::endl;
            break;
        }

        //wait 20 ms between successive frames and break the loop if key q is pressed
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }


    }
    // Release the video capture object
    cap.release();
    cv::destroyAllWindows();
    return 0;

}
