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

            double tilt_angle_0 = std::atan2(p[0].y - r.y, p[0].x - r.x);
            double tilt_angle_1 = std::atan2(p[1].y - r.y, p[1].x - r.x);

            tilt_angle_0 = tilt_angle_0 / CV_PI * 180;
            tilt_angle_1 = tilt_angle_1 / CV_PI * 180;


            if (tilt_angle_0 < 0) {
                tilt_angle_0 = -tilt_angle_0;
            } else {
                tilt_angle_0 = 360 - tilt_angle_0;
            }
            if (tilt_angle_1 < 0) {
                tilt_angle_1 = -tilt_angle_1;
            } else {
                tilt_angle_1 = 360 - tilt_angle_1;
            }

            if (tilt_angle_0 < 90 && tilt_angle_1 > 270) {
                corner_point_0 = p[0];
                corner_point_1 = p[1];
            } else if (tilt_angle_1 < 90 && tilt_angle_0 > 270) {
                corner_point_1 = p[0];
                corner_point_0 = p[1];
            } else if (tilt_angle_0 > tilt_angle_1) {
                corner_point_1 = p[1];
                corner_point_0 = p[0];
            } else if (tilt_angle_0 < tilt_angle_1) {
                corner_point_0 = p[1];
                corner_point_1 = p[0];
            }

            cv::Point2f center = (corner_point_0 + corner_point_1) / 2;
            tilt_angle = std::atan2(center.y - r.y, center.x - r.x);
            tilt_angle = tilt_angle / CV_PI * 180;
            if (tilt_angle < 0) {
                tilt_angle = -tilt_angle;
            } else {
                tilt_angle = 360 - tilt_angle;
            }
        }

        cv::Point2f p[4];
        cv::Point2f corner_point_0;
        cv::Point2f corner_point_1;
        double tilt_angle = 0;
    };

    struct NearEnd : public cv::RotatedRect {
        NearEnd() = default;

        NearEnd(const cv::RotatedRect &box, bool activated, const cv::Point2f r) : cv::RotatedRect(box) {
            is_activated = activated;
            box.points(p);

            std::sort(p, p + 4,
                      [&r](const cv::Point2f &a, const cv::Point2f &b) { return cv::norm(a - r) < cv::norm(b - r); });

            double tilt_angle_2 = std::atan2(p[2].y - r.y, p[2].x - r.x);
            double tilt_angle_3 = std::atan2(p[3].y - r.y, p[3].x - r.x);
            tilt_angle_2 = tilt_angle_2 / CV_PI * 180;
            tilt_angle_3 = tilt_angle_3 / CV_PI * 180;

            if (tilt_angle_2 < 0) {
                tilt_angle_2 = -tilt_angle_2;
            } else {
                tilt_angle_2 = 360 - tilt_angle_2;
            }
            if (tilt_angle_3 < 0) {
                tilt_angle_3 = -tilt_angle_3;
            } else {
                tilt_angle_3 = 360 - tilt_angle_3;
            }

            if (tilt_angle_2 < 90 && tilt_angle_3 > 270) {
                corner_point_2 = p[3];
                corner_point_3 = p[2];
            } else if (tilt_angle_3 < 90 && tilt_angle_2 > 270) {
                corner_point_3 = p[3];
                corner_point_2 = p[2];
            } else if (tilt_angle_2 > tilt_angle_3) {
                corner_point_3 = p[2];
                corner_point_2 = p[3];
            } else if (tilt_angle_2 < tilt_angle_3) {
                corner_point_2 = p[2];
                corner_point_3 = p[3];
            }

            cv::Point2f center = (corner_point_2 + corner_point_3) / 2;
            tilt_angle = std::atan2(center.y - r.y, center.x - r.x);
            tilt_angle = tilt_angle / CV_PI * 180;

            if (tilt_angle < 0) {
                tilt_angle = -tilt_angle;
            } else {
                tilt_angle = 360 - tilt_angle;
            }
        }

        cv::Point2f p[4];
        cv::Point2f corner_point_2;
        cv::Point2f corner_point_3;
        double tilt_angle = 0;
        bool is_activated = false;
    };

    struct Blade {
        Blade() = default;

        Blade(const NearEnd &near_end, const FarEnd &far_end) {
            top_left = far_end.corner_point_0;
            top_right = far_end.corner_point_1;
            bottom_right = near_end.corner_point_2;
            bottom_left = near_end.corner_point_3;
            center = ((top_right + top_left) / 2 + (bottom_left + bottom_right) / 2) / 2;

            is_activated = near_end.is_activated;
            if(abs(far_end.tilt_angle - near_end.tilt_angle)>180)
                tilt_angle = (far_end.tilt_angle + near_end.tilt_angle + 360) / 2;
            else
                tilt_angle = (far_end.tilt_angle + near_end.tilt_angle) / 2;
        }

        //以扇叶远端为上
        cv::Point2f top_left;
        cv::Point2f top_right;
        cv::Point2f bottom_right;
        cv::Point2f bottom_left;

        cv::Point2f center;
        double tilt_angle = 0;
        bool is_activated = false;
    };

}// namespace rm_power_rune
#endif //POWER_RUNE_BLADE_H
