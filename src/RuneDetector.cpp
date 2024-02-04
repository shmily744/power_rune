#include "RuneDetector.h"

rm_power_rune::RuneDetector::RuneDetector(const int &bin_thres, const int &channel_thres, const int &color)
        : binary_thres(bin_thres),
          channel_thres(channel_thres), detect_color(color)
          {}

std::vector<rm_power_rune::Blade> rm_power_rune::RuneDetector::detect(const cv::Mat &input) {

    preprocessImage(input);

    return {};
}

cv::Mat rm_power_rune::RuneDetector::preprocessImage(const cv::Mat &input) const {

    cv::Mat subtracted_img, gray_img, threshold_img, binary_img;

    cvtColor(input, gray_img, cv::COLOR_BGR2GRAY);
    threshold(gray_img, threshold_img, binary_thres, 255, cv::THRESH_BINARY);
    //通道相减 突出识别颜色
    std::vector<cv::Mat> channels;
    split(input, channels);
    if (detect_color == RED)
        subtract(channels[2], channels[0], subtracted_img);
    else if (detect_color == BLUE)
        subtract(channels[0], channels[2], subtracted_img);

    //图像预处理参数 二值化、膨胀、形态学闭操作
    threshold(subtracted_img, subtracted_img, channel_thres, 255, cv::THRESH_BINARY);

    //与操作消除白色灯条与蓝色灯条相连部分
    binary_img = subtracted_img & threshold_img;

    cv::Mat kernel = cv::getStructuringElement(0, cv::Size(3, 3));
    dilate(binary_img, binary_img, kernel, cv::Point(-1, -1), 1);

    kernel = cv::getStructuringElement(0, cv::Size(7, 7));
    morphologyEx(binary_img, binary_img, cv::MORPH_CLOSE, kernel);


    cv::namedWindow("binary_img");
    cv::resizeWindow("binary_img", 740, 620);
    imshow("binary_img", binary_img);

    return binary_img;
}

