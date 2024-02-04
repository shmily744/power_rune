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


        int binary_thres;
        int channel_thres;
        int detect_color;

    };


}


#endif //POWER_RUNE_RUNEDETECTOR_H
