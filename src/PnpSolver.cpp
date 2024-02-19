// Copyright 2022 Chen Jun

#include "PnpSolver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_power_rune {
    PnPSolver::PnPSolver(
            const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
            : camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
              dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()) {

        //Todo: 世界坐标系待修改
        // Unit: m
        constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
        constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;

        // Start from bottom left in clockwise order
        // Model coordinate: x forward, y left, z up
        armor_points_.emplace_back(0, small_half_y, -small_half_z);
        armor_points_.emplace_back(0, small_half_y, small_half_z);
        armor_points_.emplace_back(0, -small_half_y, small_half_z);
        armor_points_.emplace_back(0, -small_half_y, -small_half_z);
    }

    bool PnPSolver::solvePnP(const Blade &blade, cv::Mat &rvec, cv::Mat &tvec) {
        std::vector<cv::Point2f> image_armor_points;

        // Fill in image points
        image_armor_points.emplace_back(blade.bottom_left);
        image_armor_points.emplace_back(blade.top_left);
        image_armor_points.emplace_back(blade.bottom_right);
        image_armor_points.emplace_back(blade.bottom_left);

        // Solve pnp
        auto object_points  = armor_points_ ;
        return cv::solvePnP(
                object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
                cv::SOLVEPNP_IPPE);
    }

    float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) {
        auto cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
        auto cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
        return static_cast<float>(cv::norm(image_point - cv::Point2f(cx, cy)));
    }

}  // namespace rm_auto_aim
