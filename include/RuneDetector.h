#ifndef POWER_RUNE_RUNEDETECTOR_H
#define POWER_RUNE_RUNEDETECTOR_H

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

#include "Blade.h"
#include "AdamFitter.h"

namespace rm_power_rune {
    class RuneDetector {
    public:
        RuneDetector(const int & bin_thres, const int & channel_thres, const int & color, const int & num_points);

        std::vector<cv::Point> detect(const cv::Mat & input);

        cv::Mat preprocessImage(const cv::Mat & input) const;
        std::pair<std::vector<FarEnd>, std::vector<NearEnd>> findRAndEnds(const cv::Mat & bin_img);
        static std::vector<Blade> matchEnds(std::vector<FarEnd> &far_ends, std::vector<NearEnd> &near_ends);
        void getAngularV(const std::vector<Blade>& blades);
        static void rotatePoints(const cv::Point& center, std::vector<cv::Point>& Points, double angle);

        int binary_thres;
        int channel_thres;
        int detect_color;
        int num_points;

        // For debug usage
        cv::Mat binary_img;

        void drawEnds(cv::Mat & img);
        int drawResults(cv::Mat & img);


    private:
        int width_;
        int height_;

        double distance_r_to_center_;
        double t_;
        double time_point_;

        bool is_convergence_;
        std::vector<cv::Point> target_points_;

        std::vector<double> vec_t_;
        std::vector<double> angular_v_;
        fitter::AdamFitter adam_fitter_;

        cv::Point2f r_;
        std::pair<std::vector<FarEnd>, std::vector<NearEnd>> ends_;
        std::vector<FarEnd>  far_ends_;
        std::vector<NearEnd>  near_ends_;
        std::vector<Blade> blades_;
        std::vector<double> last_tilt_angles_;
    };
}
#endif //POWER_RUNE_RUNEDETECTOR_H
