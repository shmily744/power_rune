//
// Created by sph on 2024/1/4.
//

#ifndef ADAMFITTING_ADAMFITTER_H
#define ADAMFITTING_ADAMFITTER_H

#include<Eigen/Dense>
#include <vector>

namespace fitter {

    class AdamFitter {
    private:
        double a_;
        double omega_;
        double phi_;
        double b_;
        int num_iterations_;

        double beta1_ = 0.9;
        double beta2_ = 0.999;
        double learning_rate_ = 0.1;
        double epsilon_ = 1e-8;
        double convergence_threshold_ = 1e-6;

        //一阶矩和二阶矩
        double m_a_;
        double v_a_;
        double m_omega_;
        double v_omega_;
        double m_phi_;
        double v_phi_;
        double m_b_;
        double v_b_;

        static double loss(const Eigen::ArrayXd &angular_velocity, const Eigen::ArrayXd &angular_velocity_pred);

    public:
        explicit AdamFitter(double a = 0.9125,
                            double omega = 1.9420,
                            double phi = 1.0707,
                            double b = 1.1775,
                            int num_iterations = 1000,
                            double beta1 = 0.9,
                            double beta2 = 0.999,
                            double learning_rate = 0.1,
                            double epsilon = 1e-6,
                            double convergence_threshold = 1e-5,
                            double m_a = 0.0,
                            double v_a = 0.0,
                            double m_omega = 0.0,
                            double v_omega = 0.0,
                            double m_phi = 0.0,
                            double v_phi = 0.0,
                            double m_b = 0.0,
                            double v_b = 0.0);

        bool Fitting(const Eigen::ArrayXd &t, const Eigen::ArrayXd &angular_velocity, std::vector<double> *result);
        static Eigen::ArrayXd sine_function(const Eigen::ArrayXd &t, double a, double omega, double phi, double b);

    };
} // fitter

#endif //ADAMFITTING_ADAMFITTER_H
