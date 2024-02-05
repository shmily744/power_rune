#include "RuneDetector.h"

namespace rm_power_rune {
    RuneDetector::RuneDetector(const int &bin_thres, const int &channel_thres, const int &color)
            : binary_thres(bin_thres),
              channel_thres(channel_thres), detect_color(color) {
        r_ = cv::Point2f(0, 0);
        width = 0;
        height = 0;
    }

    std::vector<rm_power_rune::Blade> rm_power_rune::RuneDetector::detect(const cv::Mat &input) {
        if(width == 0&&height == 0){
            width = input.cols;
            height = input.rows;
        }

        binary_img = preprocessImage(input);
        findRAndEnds(input, binary_img);

        return {};
    }

    cv::Mat rm_power_rune::RuneDetector::preprocessImage(const cv::Mat &input) const {

        cv::Mat subtracted_img, gray_img, threshold_img;

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

        cv::Mat bin_img;
        //与操作消除白色灯条与蓝色灯条相连部分
        bin_img = (subtracted_img & threshold_img);


        cv::Mat kernel = cv::getStructuringElement(0, cv::Size(3, 3));
        dilate(bin_img, bin_img, kernel, cv::Point(-1, -1), 1);

        kernel = cv::getStructuringElement(0, cv::Size(7, 7));
        morphologyEx(bin_img, bin_img, cv::MORPH_CLOSE, kernel);


        cv::namedWindow("binary_img");
        cv::resizeWindow("binary_img", 740, 620);
        imshow("binary_img", bin_img);

        return bin_img;
    }

    void RuneDetector::findRAndEnds(const cv::Mat &rbg_img, const cv::Mat &binary_img) {
        using std::vector;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        drawContours(rbg_img, contours, -1, cv::Scalar(0, 255, 0), 1);
        double distance_r_to_center;

        for (const auto &contour: contours) {
            if (contour.size() < 5) continue;

            auto r_rect = cv::minAreaRect(contour);

            float area_rect = r_rect.size.area();
            if (area_rect >= 10000.0 || area_rect < 80) continue;

            float width_rect;
            float height_rect;
            //判断长宽比
            if (r_rect.size.width > r_rect.size.height) {
                width_rect = r_rect.size.width;
                height_rect = r_rect.size.height;
            } else {
                width_rect = r_rect.size.height;
                height_rect = r_rect.size.width;
            }

            if (width_rect / height_rect < 4) {
                if (area_rect > 100 && area_rect < 500) { // find R
                    if (height_rect / width_rect > 0.85) {
                        if (r_.x == 0 && r_.y == 0) {
                            r_ = r_rect.center;
                            distance_r_to_center = cv::norm(r_ - cv::Point2f(width / 2, height / 2));
                        } else {
                            double distance = cv::norm(r_rect.center - cv::Point2f(width / 2, height / 2));
                            if (abs(distance - distance_r_to_center) < 1000) {
                                r_ = r_rect.center;
                                distance_r_to_center = distance;
                            }
                        }
                    }
                } else {    //find ends
                    // find NearEnd
                    if (area_rect > 4000 && area_rect < 6500&&width_rect / height_rect > 0.9 && width_rect / height_rect < 1.6) {
                        if (1) {
                            auto near_end = NearEnd(r_rect, false);
                            near_ends_.emplace_back(near_end);
                        }
                    }

                    // find FarEnd
                    if (area_rect > 1100 && area_rect < 1500 && width_rect / height_rect > 2.1 && width_rect / height_rect < 2.9) {
                        auto far_end = FarEnd(r_rect);
                        far_ends_.emplace_back(far_end);
                    }
                }
            }
        }
    }

    std::vector<Blade> rm_power_rune::RuneDetector::matchEnds(const std::vector<FarEnd> &lights, std::vector<NearEnd>) {
        return std::vector<Blade>();
    }

    void RuneDetector::drawResults(cv::Mat &img) {
        cv::circle(img, r_, 15, cv::Scalar(0, 255, 255));
    }
}


