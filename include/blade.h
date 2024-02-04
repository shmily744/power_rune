#ifndef POWER_RUNE_BLADE_H
#define POWER_RUNE_BLADE_H

#include <opencv2/opencv.hpp>

namespace rm_power_rune {
    const int RED = 0;
    const int BLUE = 1;

    struct FarEnd : public cv::RotatedRect {
        FarEnd() = default;

        explicit FarEnd(cv::RotatedRect box) : cv::RotatedRect(box) {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        int color;
        cv::Point2f top, bottom;
        double length;
        double width;
        float tilt_angle;
    };

    struct NearEnd : public cv::RotatedRect {
        NearEnd() = default;

        explicit NearEnd(cv::RotatedRect box) : cv::RotatedRect(box) {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        int color;
        cv::Point2f top, bottom;
        double length;
        double width;
        float tilt_angle;
    };

    struct Blade {
        Blade() = default;

        Blade(const FarEnd &far_end, const NearEnd &near_end) {

        }

        //以扇叶远端为上
        cv::Point2f top_left;
        cv::Point2f top_right;
        cv::Point2f bottom_right;
        cv::Point2f bottom_left;

        cv::Point2f center;
        bool is_activated;
    };

}// namespace rm_power_rune
#endif //POWER_RUNE_BLADE_H
