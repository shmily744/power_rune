//
// Created by sph on 2024/1/4.
//

#include <iostream>
#include "AdamFitter.h"

namespace fitter {
    AdamFitter::AdamFitter(double a, double omega, double phi, double b, int num_iterations, double beta1, double beta2,
                           double learning_rate, double epsilon, double convergence_threshold, double m_a, double v_a,
                           double m_omega, double v_omega, double m_phi, double v_phi, double m_b, double v_b) :
            a_(a),
            omega_(omega),
            phi_(phi),
            b_(b),
            num_iterations_(num_iterations),
            beta1_(beta1),
            beta2_(beta2),
            learning_rate_(learning_rate),
            epsilon_(epsilon),
            convergence_threshold_(convergence_threshold),
            m_a_(m_a),
            v_a_(v_a),
            m_omega_(m_omega),
            v_omega_(v_omega),
            m_phi_(m_phi),
            v_phi_(v_phi),
            m_b_(m_b),
            v_b_(v_b) {}

    Eigen::ArrayXd fitter::AdamFitter::sine_function(const Eigen::ArrayXd &t, double a,
                                                     double omega, double phi, double b) {
        Eigen::ArrayXd result = a * (omega * t + phi).array().sin() + b;
        return result;
    }

    double AdamFitter::loss(const Eigen::ArrayXd &angular_velocity, const Eigen::ArrayXd &angular_velocity_pred) {
        double result = (angular_velocity - angular_velocity_pred).square().mean();
        return result;
    }

    bool AdamFitter::Fitting(const Eigen::ArrayXd &t, const Eigen::ArrayXd &angular_velocity,
                             std::vector<double> *result) {

        for (int i = 0; i <= num_iterations_; i++) {
            //计算预测值、损失
            Eigen::ArrayXd angular_velocity_pred = sine_function(t, a_, omega_, phi_, b_);
            double current_loss = loss(angular_velocity, angular_velocity_pred);

            //计算参数梯度
            Eigen::ArrayXd temp = 2 * (angular_velocity_pred - angular_velocity);
            Eigen::ArrayXd temp_angle = omega_ * t + phi_;
            double grad_a = (temp * temp_angle.sin()).mean();
            double grad_omega = (temp * a_ * t * temp_angle.cos()).mean();
            double grad_phi = (temp * a_ * temp_angle.cos()).mean();
            double grad_b = temp.mean();

            //更新一阶矩
            m_a_ = beta1_ * m_a_ + (1 - beta1_) * grad_a;
            m_omega_ = beta1_ * m_omega_ + (1 - beta1_) * grad_omega;
            m_phi_ = beta1_ * m_phi_ + (1 - beta1_) * grad_phi;
            m_b_ = beta1_ * m_b_ + (1 - beta1_) * grad_b;

            //更新二阶矩
            v_a_ = beta2_ * v_a_ + (1 - beta2_) * grad_a * grad_a;
            v_omega_ = beta2_ * v_omega_ + (1 - beta2_) * grad_omega * grad_omega;
            v_phi_ = beta2_ * v_phi_ + (1 - beta2_) * grad_phi * grad_phi;
            v_b_ = beta2_ * v_b_ + (1 - beta2_) * grad_b * grad_b;

            //修正一阶矩和二阶矩的偏差
            double m_a_hat = m_a_ / (1 - pow(beta1_, (i + 1)));
            double m_omega_hat = m_omega_ / (1 - pow(beta1_, (i + 1)));
            double m_phi_hat = m_phi_ / (1 - pow(beta1_, (i + 1)));
            double m_b_hat = m_b_ / (1 - pow(beta1_, (i + 1)));
            double v_a_hat = v_a_ / (1 - pow(beta2_, (i + 1)));
            double v_omega_hat = v_omega_ / (1 - pow(beta2_, (i + 1)));
            double v_phi_hat = v_phi_ / (1 - pow(beta2_, (i + 1)));
            double v_b_hat = v_b_ / (1 - pow(beta2_, (i + 1)));

            //更新参数
            a_ = std::min(std::max(a_ - learning_rate_ * m_a_hat / (std::sqrt(v_a_hat) + epsilon_), 0.780), 1.045);
            omega_ = std::min(
                    std::max(omega_ - learning_rate_ * m_omega_hat / (std::sqrt(v_omega_hat) + epsilon_), 1.884),
                    1.980);
            phi_ = std::min(std::max(phi_ - learning_rate_ * m_phi_hat / (std::sqrt(v_phi_hat) + epsilon_), 0.000),
                            3.1415926);
            b_ -= learning_rate_ * m_b_hat / (std::sqrt(v_b_hat) + epsilon_);
            b_ = 0.8 * (b_ - learning_rate_ * m_b_hat / (std::sqrt(v_b_hat) + epsilon_)) + 0.2 * (2.090 - a_);
            //新的预测值
            Eigen::ArrayXd angular_velocity_hat = sine_function(t, a_, omega_, phi_, b_);

            //判断收敛
            if (std::abs(current_loss - loss(angular_velocity, angular_velocity_hat)) < convergence_threshold_) {
                std::cout << "Converged after " << i + 1 << " iterations" << std::endl;
                result->emplace_back(a_);
                result->emplace_back(omega_);
                result->emplace_back(phi_);
                result->emplace_back(b_);
                return true;
            }
        }
        result->emplace_back(a_);
        result->emplace_back(omega_);
        result->emplace_back(phi_);
        result->emplace_back(b_);
        return false;
    }
} // fitter
