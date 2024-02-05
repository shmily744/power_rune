#ifndef POWER_RUNE_RUNEDETECTOR_H
#define POWER_RUNE_RUNEDETECTOR_H

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "blade.h"

namespace rm_power_rune {
    class RuneDetector {
    public:
        RuneDetector(const int & bin_thres, const int & channel_thres, const int & color);

        std::vector<Blade> detect(const cv::Mat & input);

        cv::Mat preprocessImage(const cv::Mat & input) const;
        void findRAndEnds(const cv::Mat & rbg_img, const cv::Mat & binary_img);
        std::vector<Blade> matchEnds(const std::vector<FarEnd> & lights, std::vector<NearEnd>);

        int binary_thres;
        int channel_thres;
        int detect_color;

        // For debug usage
        cv::Mat binary_img;
        void drawResults(cv::Mat & img);

    private:
        int width;
        int height;
        cv::Point2f r_;
        std::vector<FarEnd>  far_ends_;
        std::vector<NearEnd>  near_ends_;
        std::vector<Blade> blades_;

    };


}


#endif //POWER_RUNE_RUNEDETECTOR_H
