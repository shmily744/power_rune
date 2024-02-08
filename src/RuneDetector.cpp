#include "RuneDetector.h"

namespace rm_power_rune {
    RuneDetector::RuneDetector(const int &bin_thres, const int &channel_thres, const int &color)
            : binary_thres(bin_thres),
              channel_thres(channel_thres), detect_color(color) {
        r_ = cv::Point2f(0, 0);
        width_ = 0;
        height_ = 0;
        distance_r_to_center_ = cv::norm(
                r_ - cv::Point2f(static_cast<float>(width_) / 2, static_cast<float>(height_) / 2));
    }

    std::vector<rm_power_rune::Blade> rm_power_rune::RuneDetector::detect(const cv::Mat &input) {
        if (width_ == 0 && height_ == 0) {
            width_ = input.cols;
            height_ = input.rows;
        }

        binary_img = preprocessImage(input);
        ends_ = findRAndEnds(binary_img);
        far_ends_ = ends_.first;
        near_ends_ = ends_.second;

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


        cv::Mat kernel = cv::getStructuringElement(0, cv::Size(2, 2));
        dilate(bin_img, bin_img, kernel, cv::Point(-1, -1), 1);

        kernel = cv::getStructuringElement(0, cv::Size(7, 7));
        morphologyEx(bin_img, bin_img, cv::MORPH_CLOSE, kernel);

        return bin_img;
    }

    std::pair<std::vector<FarEnd>, std::vector<NearEnd>> RuneDetector::findRAndEnds(const cv::Mat &bin_img) {
        std::vector<FarEnd> far_ends;
        std::vector<NearEnd> near_ends;

        using std::vector;

        //轮廓及其层次结构
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;

        cv::findContours(bin_img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        int contour_index = 0;
        for (const auto &contour: contours) {

            //只检测5个点以上且最外层轮廓
            if (contour.size() < 5) {
                contour_index++;
                continue;
            }
            if (hierarchy[contour_index][3] != -1) {
                contour_index++;
                continue;
            }


            auto r_rect = cv::minAreaRect(contour);

            float area_rect = r_rect.size.area();
            if (area_rect >= 10000.0 || area_rect < 80) {
                contour_index++;
                continue;
            }

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
                if (area_rect > 80 && area_rect < 600) { // find R
                    if (height_rect / width_rect > 0.85) {
                        if (r_.x == 0 && r_.y == 0) {
                            r_ = r_rect.center;
                            distance_r_to_center_ = cv::norm(
                                    r_ - cv::Point2f(static_cast<float>(width_) / 2, static_cast<float>(height_) / 2));
                        } else {
                            double distance = cv::norm(r_rect.center - cv::Point2f(static_cast<float>(width_) / 2,
                                                                                   static_cast<float>(height_) / 2));
                            if (abs(distance - distance_r_to_center_) < 100) {
                                r_ = r_rect.center;
                                distance_r_to_center_ = distance;
                            } else std::cout << "R jump!" << std::endl;
                        }
                    }
                } else {    //find ends
                    // find NearEnd
                    if (area_rect > 4000 && area_rect < 6500 && width_rect / height_rect > 0.9 &&
                        width_rect / height_rect < 1.6) {
                        if (hierarchy[contour_index][2] != -1) {
                            auto near_end = NearEnd(r_rect, true, r_);
                            near_ends.emplace_back(near_end);
                        } else {
                            auto near_end = NearEnd(r_rect, false, r_);
                            near_ends.emplace_back(near_end);
                        }
                    }

                    // find FarEnd
                    if (area_rect > 800 && area_rect < 1500 && width_rect / height_rect > 1.8 &&
                        width_rect / height_rect < 3.5) {
                        auto far_end = FarEnd(r_rect, r_);
                        far_ends.emplace_back(far_end);
                    }
                }
            }
            contour_index++;
        }
        return std::make_pair(far_ends, near_ends);
    }

    std::vector<Blade>
    rm_power_rune::RuneDetector::matchEnds(const std::vector<FarEnd> &far_ends, const std::vector<NearEnd> near_ends) {
        return std::vector<Blade>();
    }

    int RuneDetector::drawResults(cv::Mat &img) {
        using std::vector;

        //draw R
        cv::circle(img, r_, 15, cv::Scalar(0, 255, 255));
        //std::cout << r_ << std::endl;
        std::cout<<"##################"<<std::endl;
        //draw far_ends
        for (auto far_end: far_ends_) {
            vector<cv::Point> p(far_end.p, far_end.p + 4);
            vector<vector<cv::Point>> temp;
            temp.emplace_back(p);
            drawContours(img, temp, -1, cv::Scalar(0, 0, 255), 2);
            cv::circle(img, far_end.corner_point_0, 5, cv::Scalar(0, 255, 255));
            cv::circle(img, far_end.corner_point_1, 5, cv::Scalar(0, 255, 255));
            std::cout<<"far: "<<far_end.tilt_angle<<std::endl;
        }

        //draw near_ends
        for (auto near_end: near_ends_) {
            vector<cv::Point> p(near_end.p, near_end.p + 4);
            vector<vector<cv::Point>> temp;
            temp.emplace_back(p);
            if (near_end.is_activated)
                drawContours(img, temp, -1, cv::Scalar(0, 255, 0), 2);
            else
                drawContours(img, temp, -1, cv::Scalar(0, 0, 255), 2);

            cv::circle(img, near_end.corner_point_2, 5, cv::Scalar(0, 255, 255));
            cv::circle(img, near_end.corner_point_3, 5, cv::Scalar(0, 255, 255));
            std::cout<<"near: "<<near_end.tilt_angle<<std::endl;
        }
        std::cout<<"##################"<<std::endl;
        imshow("RawImage", img);
        imshow("BinaryImage", binary_img);

        int key = cv::waitKey(1);
        return key;
    }
}



