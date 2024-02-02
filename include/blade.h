#ifndef POWER_RUNE_BLADE_H
#define POWER_RUNE_BLADE_H

#include <opencv2/opencv.hpp>

namespace rm_power_rune {
    const int RED = 0;
    const int BLUE = 1;

    struct TargetBlade {
        TargetBlade() = default;

        TargetBlade(const cv::Point2f &P1, const cv::Point2f &P2, const cv::Point2f &P3, const cv::Point2f &P4) {
            upper_left = P1;
            upper_right = P2;
            lower_right = P3;
            lower_left = P4;

            center.x = ((upper_left.x + upper_right.x) / 2 + (lower_left.x + lower_right.x) / 2) / 2;
            center.y = ((upper_left.y + upper_right.y) / 2 + (lower_left.y + lower_right.y) / 2) / 2;
        }

        //以扇叶远端为上
        cv::Point2f upper_left;
        cv::Point2f upper_right;
        cv::Point2f lower_right;
        cv::Point2f lower_left;

        cv::Point2f center;
    };

}// namespace rm_power_rune
#endif //POWER_RUNE_BLADE_H
