#include "RuneDetector.h"

namespace rm_power_rune {
    RuneDetector::RuneDetector(const int &bin_thres, const int &channel_thres, const int &color, const int &num_points)
            : binary_thres(bin_thres), channel_thres(channel_thres),
              detect_color(color), num_points(num_points) {
        r_ = cv::Point2f(0, 0);
        width_ = 0;
        height_ = 0;

        t_ = 0;
        is_convergence_ = false;

        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = now.time_since_epoch();
        time_point_ = duration.count();

        distance_r_to_center_ = cv::norm(
                r_ - cv::Point2f(static_cast<float>(width_) / 2, static_cast<float>(height_) / 2));
    }

    std::vector<cv::Point> rm_power_rune::RuneDetector::detect(const cv::Mat &input) {
        if (width_ == 0 && height_ == 0) {
            width_ = input.cols;
            height_ = input.rows;
        }

        binary_img = preprocessImage(input);
        ends_ = findRAndEnds(binary_img);
        far_ends_ = ends_.first;
        near_ends_ = ends_.second;
        blades_ = matchEnds(far_ends_, near_ends_);
        getAngularV(blades_);

        Eigen::ArrayXd t(num_points);
        Eigen::ArrayXd v;
        Eigen::ArrayXd y_pre;
        std::vector<double> result;

        target_points_.clear();

        if (!vec_t_.empty() || !angular_v_.empty()) {

            t = Eigen::Map<Eigen::ArrayXd>(vec_t_.data(), static_cast<Eigen::Index>(vec_t_.size()));

            v = Eigen::Map<Eigen::ArrayXd>(angular_v_.data(), static_cast<Eigen::Index>(angular_v_.size()));
            is_convergence_ = adam_fitter_.Fitting(t, v, &result);
            if (is_convergence_) {
                double a = result[0];
                double omega = result[1];
                double phi = result[2];
                double b = result[3];

                double angle = (a * sin((omega * (vec_t_.back()) + phi)) + b)*0.1;

                for(const auto& blade:blades_){
                    if(!blade.is_activated){

                        target_points_.emplace_back(blade.top_left);
                        target_points_.emplace_back(blade.top_right);
                        target_points_.emplace_back(blade.bottom_right);
                        target_points_.emplace_back(blade.bottom_left);

                        rotatePoints(r_, target_points_, -angle);
                    }
                }
            }
        }
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
                    if (height_rect / width_rect > 0.8) {
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
    rm_power_rune::RuneDetector::matchEnds(std::vector<FarEnd> &far_ends, std::vector<NearEnd> &near_ends) {
        std::vector<Blade> blades;

        if (far_ends.size() != near_ends.size()) {
            std::cout << "Number mismatch between far_ends and near_ends" << std::endl;
        }

        std::sort(far_ends.begin(), far_ends.end(),
                  [](const FarEnd &a, const FarEnd &b) { return a.tilt_angle < b.tilt_angle; });
        std::sort(near_ends.begin(), near_ends.end(),
                  [](const NearEnd &a, const NearEnd &b) { return a.tilt_angle < b.tilt_angle; });

        for (const auto &near_end: near_ends) {
            for (const auto &far_end: far_ends) {
                if ((near_end.tilt_angle < 3 && far_end.tilt_angle > 357) ||
                    (far_end.tilt_angle < 3 && near_end.tilt_angle > 357)) {
                    Blade blade(near_end, far_end);
                    blades.emplace_back(blade);
                    break;
                }
                if (abs(near_end.tilt_angle - far_end.tilt_angle) < 5) {
                    Blade blade(near_end, far_end);
                    blades.emplace_back(blade);
                    break;
                }
            }
        }
        return blades;
    }

    void RuneDetector::getAngularV(const std::vector<Blade> &blades) {
        std::vector<double> delta_angles;

        if (last_tilt_angles_.empty()) {
            for (const auto &blade: blades) {
                last_tilt_angles_.emplace_back(blade.tilt_angle);
            }
            auto now = std::chrono::system_clock::now();
            std::chrono::duration<double> duration = now.time_since_epoch();
            time_point_ = duration.count();
            t_ = 0;
            angular_v_.clear();
            vec_t_.clear();

        } else {
            double delta_angle;
            for (double last_tilt_angle: last_tilt_angles_) {
                for (const auto &blade: blades) {
                    if (last_tilt_angle > 355 && blade.tilt_angle < 5) {
                        delta_angle = 360 - last_tilt_angle + blade.tilt_angle;
                        delta_angles.push_back(delta_angle);
                    } else if (abs(blade.tilt_angle - last_tilt_angle) < 5) {
                        delta_angle =
                                (blade.tilt_angle - last_tilt_angle) > 0 ? (blade.tilt_angle - last_tilt_angle) : 0.01;
                        delta_angles.push_back(delta_angle);
                    } else if (blades.size() == 1) {
                        std::cout << "blade jump" << std::endl;
                    }
                }
            }
            last_tilt_angles_.clear();
            for (const auto &blade: blades) {
                last_tilt_angles_.emplace_back(blade.tilt_angle);
            }
            double sum = 0;

            for (auto delta_a: delta_angles) {
                sum += delta_a;
            }
            double delta_angle_avg = (sum / static_cast<double> (delta_angles.size())) * CV_PI / 180;

            auto now = std::chrono::system_clock::now();
            std::chrono::duration<double> duration = now.time_since_epoch();
            double delta_t = duration.count() - time_point_;
            time_point_ = duration.count();
            t_ += delta_t;

            double angular_v;
            angular_v = delta_angle_avg / delta_t < 2.09 ? delta_angle_avg / delta_t : 2.09;

            if (vec_t_.size() < num_points) {
                vec_t_.emplace_back(t_);
                angular_v_.emplace_back(angular_v);
            } else {
                vec_t_.erase(vec_t_.begin());
                angular_v_.erase(angular_v_.begin());
                vec_t_.emplace_back(t_);
                angular_v_.emplace_back(angular_v);
            }
        }
    }

    void RuneDetector::rotatePoints(const cv::Point &center, std::vector<cv::Point> &Points, double angle) {
        // 对每个点执行旋转,其实对输入点顺序并无要求。。。仅是对输入的所有点绕center做相同角度的旋转
        Eigen::Vector2d eigenCenter(center.x, center.y);
        Eigen::Vector2d LeftTopPoint(Points[0].x, (Points[0].y));
        Eigen::Vector2d RightTopPoint(Points[1].x, (Points[1].y));
        Eigen::Vector2d RightLowPoint(Points[2].x, (Points[2].y));
        Eigen::Vector2d LeftLowTopPoint(Points[3].x, (Points[3].y));

        Eigen::Transform<double, 2, Eigen::Affine> transform;
        transform = Eigen::Translation2d(eigenCenter) * Eigen::Rotation2Dd(angle) * Eigen::Translation2d(-eigenCenter);

        LeftTopPoint = transform * LeftTopPoint;
        RightTopPoint = transform * RightTopPoint;
        RightLowPoint = transform * RightLowPoint;
        LeftLowTopPoint = transform * LeftLowTopPoint;

        Points.clear();
        Points.emplace_back(LeftTopPoint.x(), LeftTopPoint.y());
        Points.emplace_back(RightTopPoint.x(), RightTopPoint.y());
        Points.emplace_back(RightLowPoint.x(), RightLowPoint.y());
        Points.emplace_back(LeftLowTopPoint.x(), LeftLowTopPoint.y());
    }

    void RuneDetector::drawEnds(cv::Mat &img) {
        using std::vector;
        std::cout << "##################" << std::endl;
        //draw far_ends
        for (auto far_end: far_ends_) {
            vector<cv::Point> p(far_end.p, far_end.p + 4);
            vector<vector<cv::Point>> temp;
            temp.emplace_back(p);
            drawContours(img, temp, -1, cv::Scalar(0, 0, 255), 2);
            cv::circle(img, far_end.corner_point_0, 5, cv::Scalar(0, 255, 255));
            cv::circle(img, far_end.corner_point_1, 5, cv::Scalar(0, 255, 255));

            cv::putText(img, "0", far_end.corner_point_0, 0, 0.6, cv::Scalar(0, 255, 255));
            cv::putText(img, "1", far_end.corner_point_1, 0, 0.6, cv::Scalar(0, 255, 255));
            std::cout << "far: " << far_end.tilt_angle << std::endl;
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

            cv::putText(img, "2", near_end.corner_point_2, 0, 0.6, cv::Scalar(0, 255, 255));
            cv::putText(img, "3", near_end.corner_point_3, 0, 0.6, cv::Scalar(0, 255, 255));
            std::cout << "near: " << near_end.tilt_angle << std::endl;
        }
        std::cout << "##################" << std::endl;
    }

    int RuneDetector::drawResults(cv::Mat &img) {
        using std::vector;

        //draw R
        cv::circle(img, r_, 15, cv::Scalar(0, 255, 255));
        //std::cout << r_ << std::endl;

        //drawEnds(img);

        //draw blades
        for (const auto &blade: blades_) {
            vector<cv::Point> p;
            p.emplace_back(blade.top_left);
            p.emplace_back(blade.top_right);
            p.emplace_back(blade.bottom_right);
            p.emplace_back(blade.bottom_left);

            vector<vector<cv::Point>> temp;
            temp.emplace_back(p);

            if (blade.is_activated) {
                drawContours(img, temp, -1, cv::Scalar(0, 255, 0), 2);
                cv::circle(img, blade.center, 3, cv::Scalar(0, 255, 0), 2);
            } else {
                drawContours(img, temp, -1, cv::Scalar(0, 0, 255), 2);
                cv::circle(img, blade.center, 3, cv::Scalar(0, 0, 255), 2);

            }

        }

        if (is_convergence_ && !target_points_.empty()) {
            vector<vector<cv::Point>> a;
            a.emplace_back(target_points_);
            drawContours(img, a, -1, cv::Scalar(0, 255, 255), 2);
        }

        imshow("RawImage", img);
        //imshow("BinaryImage", binary_img);

        int key = cv::waitKey(6);
        return key;
    }
}// namespace rm_power_rune
