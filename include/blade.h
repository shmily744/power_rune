#ifndef POWER_RUNE_BLADE_H
#define POWER_RUNE_BLADE_H

#include <opencv2/opencv.hpp>

namespace rm_power_rune {
    const int RED = 0;
    const int BLUE = 1;

    struct FarEnd : public cv::RotatedRect {
        FarEnd() = default;

        explicit FarEnd(const cv::RotatedRect &box, const cv::Point2f r) : cv::RotatedRect(box) {
            box.points(p);
            std::sort(p, p + 4,
                      [&r](const cv::Point2f &a, const cv::Point2f &b) { return cv::norm(a - r) < cv::norm(b - r); });
            corner_point_0 = p[0];
            corner_point_1 = p[1];

            cv::Point2f center = (corner_point_0+corner_point_1)/2;
            tilt_angle = std::atan2(center.y - r.y, center.x - r.x);
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        cv::Point2f p[4];
        cv::Point2f corner_point_0;
        cv::Point2f corner_point_1;
        double tilt_angle = 0 ;
    };

    struct NearEnd : public cv::RotatedRect {
        NearEnd() = default;

        NearEnd(const cv::RotatedRect &box, bool activated, const cv::Point2f r) : cv::RotatedRect(box) {
            is_activated = activated;
            box.points(p);

            std::sort(p, p + 4,
                      [&r](const cv::Point2f &a, const cv::Point2f &b) { return cv::norm(a - r) < cv::norm(b - r); });
            corner_point_2 = p[2];
            corner_point_3 = p[3];

            cv::Point2f center = (corner_point_2+corner_point_3)/2;
            tilt_angle = std::atan2(center.y - r.y, center.x - r.x);
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        cv::Point2f p[4];
        cv::Point2f corner_point_2;
        cv::Point2f corner_point_3;
        double tilt_angle = 0 ;
        bool is_activated = false;
    };

    struct Blade {
        Blade() = default;

        Blade(const FarEnd &far_end, const NearEnd &near_end) {
            is_activated = near_end.is_activated;

        }

        //以扇叶远端为上
        cv::Point2f top_left;
        cv::Point2f top_right;
        cv::Point2f bottom_right;
        cv::Point2f bottom_left;

        cv::Point2f center;
        bool is_activated = false;
    };

}// namespace rm_power_rune
#endif //POWER_RUNE_BLADE_H
