#include "excitation_trajectory.hpp"

#include <Eigen/SVD>

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <libcmaes/cmaes.h>

ExcitationTrajectory::ExcitationTrajectory(const std::string& urdf_path) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    q.resize(model_.nq);
    dq.resize(model_.nv);
    ddq.resize(model_.nv);
}

void ExcitationTrajectory::generate(double period, int repeat_cnt) {
    traj_repeat_cnt             = repeat_cnt;
    trajectory_generatefinished = false;
    calc_traj(period);
    trajectory_generatefinished = best_traj != nullptr;
}
bool ExcitationTrajectory::generate_is_finished() {
    return trajectory_generatefinished;
}

bool ExcitationTrajectory::get_target_position(const rclcpp::Duration& time, Eigen::VectorXd& pos) {
    if (best_traj == nullptr) {
        return false;
    }

    return best_traj->position(std::fmod(time.seconds(), best_traj->period()), pos);
}

void ExcitationTrajectory::calc_traj(double peroid) {

    constexpr int k_harmonics           = 3;
    constexpr double k_eval_dt          = 0.01;
    constexpr int k_max_fevals          = 600;
    constexpr int k_lambda              = 24;
    constexpr double k_coeff_bound      = 2.0;
    constexpr double k_initial_sigma    = 0.7;
    constexpr double k_infeasible_cost  = 1000.0;
    constexpr double k_violation_weight = 1000.0;

    // SVD 分解，评估最小参数集
    std::random_device rd;
    std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
    std::mt19937 random_genenter(seq);

    std::uniform_real_distribution<double> pos_range(svd_pos_lower, svd_pos_upper);
    std::uniform_real_distribution<double> vel_range(-svd_vel_upper, svd_vel_upper);
    std::uniform_real_distribution<double> acc_range(-svd_acc_upper, svd_acc_upper);

    // 填写随机参数
    for (int i = 0; i < svd_Y_stack_num; i++) {
        for (int j = 0; j < model_.nv; j++) {
            q[j]   = pos_range(random_genenter);
            dq[j]  = vel_range(random_genenter);
            ddq[j] = acc_range(random_genenter);
        }

        const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
        if (i == 0) {
            Y.resize(Yi.rows() * svd_Y_stack_num, Yi.cols());
        }
        Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeFullV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    int identifiable_parameter_num_       = 0;
    for (int i = 0; i < singular_values.size(); i++) {
        if (singular_values[i] > SVD_ZERO * singular_values[0]) {
            identifiable_parameter_num_++;
        }
    }

    if (identifiable_parameter_num_ <= 0) {
        std::cerr << "No identifiable dynamic parameters found, skip excitation trajectory optimization." << std::endl;
        return;
    }

    // 3次谐波，关节个数维度，一个周期
    FourierTrajectory fourier_traj(model_.nq, k_harmonics, peroid);
    const int coefficient_num = model_.nq * 2 * fourier_traj.harmonics();
    std::vector<double> x0(coefficient_num, 0.0);
    std::vector<double> lbounds(coefficient_num, -k_coeff_bound);
    std::vector<double> ubounds(coefficient_num, k_coeff_bound);

    libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds.data(), ubounds.data(), coefficient_num);
    libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>> cma_params(x0, k_initial_sigma, k_lambda, 0, gp);
    cma_params.set_algo(aCMAES);
    cma_params.set_max_fevals(k_max_fevals);
    cma_params.set_quiet(true);
    cma_params.set_mt_feval(false);

    double best_score = -std::numeric_limits<double>::infinity();
    Eigen::MatrixXd best_mat;

    libcmaes::FitFunc objective = [this, &fourier_traj, identifiable_parameter_num_, &best_score, &best_mat, k_eval_dt, k_infeasible_cost,
                                   k_violation_weight](const double* x, const int n) {
        if (n != fourier_traj.dof() * 2 * fourier_traj.harmonics()) {
            return k_infeasible_cost;
        }

        fourier_traj.set_coefficients(x, n);

        const double violation = traj_constraint_violation(fourier_traj, k_eval_dt);
        if (violation > 0.0) {
            return k_infeasible_cost + k_violation_weight * violation;
        }

        const double score = traj_score(fourier_traj, k_eval_dt, identifiable_parameter_num_);
        if (!std::isfinite(score)) {
            return k_infeasible_cost;
        }
        return -score;
    };

    const libcmaes::CMASolutions cma_solutions = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>(objective, cma_params);
    const Eigen::VectorXd best_x               = cma_solutions.get_best_seen_candidate().get_x_pheno_dvec(cma_params);
    fourier_traj.set_coefficients(best_x);

    best_traj = std::make_shared<FourierTrajectory>(fourier_traj);
    if (std::isfinite(best_score)) {
        best_coefficients_ = best_mat;
    }
    std::cout << "best_x:" << best_x << std::endl;
}

bool ExcitationTrajectory::traj_is_available(const FourierTrajectory& traj, double dt) {
    return traj_constraint_violation(traj, dt) <= 0.0;
}

double ExcitationTrajectory::traj_constraint_violation(const FourierTrajectory& traj, double dt) {
    // 轨迹关节数与模型不一致时视为不可用
    if (traj.dof() != model_.nq) {
        return std::numeric_limits<double>::infinity();
    }

    // 傅里叶轨迹为周期轨迹，采样一个周期即可覆盖全部位置/速度取值
    const int sample_cnt = static_cast<int>(traj.period() / dt);
    double violation     = 0.0;

    for (int i = 0; i <= sample_cnt; ++i) {
        const double t = i * dt;
        q              = traj.position(t);
        dq             = traj.velocity(t);

        for (int j = 0; j < traj.dof(); ++j) {
            // 位置软限位：将关节行程按软限位比例缩放到行程中点附近
            const double lower = model_.lowerPositionLimit[j];
            const double upper = model_.upperPositionLimit[j];
            if (std::isfinite(lower) && std::isfinite(upper) && upper > lower) {
                const double mid        = 0.5 * (lower + upper);
                const double half_range = 0.5 * (upper - lower);
                const double soft_lower = mid - traj_pos_soft_limit_rate * half_range;
                const double soft_upper = mid + traj_pos_soft_limit_rate * half_range;
                const double pos_scale  = std::max(traj_pos_soft_limit_rate * half_range, 1e-6);

                if (q[j] < soft_lower) {
                    const double normalized_violation = (soft_lower - q[j]) / pos_scale;
                    violation += normalized_violation * normalized_violation;
                } else if (q[j] > soft_upper) {
                    const double normalized_violation = (q[j] - soft_upper) / pos_scale;
                    violation += normalized_violation * normalized_violation;
                }
            }

            // 关节速度超限
            const double velocity_limit = model_.velocityLimit[j];
            if (std::isfinite(velocity_limit) && velocity_limit > 0.0 && std::fabs(dq[j]) > velocity_limit) {
                const double normalized_violation = (std::fabs(dq[j]) - velocity_limit) / velocity_limit;
                violation += normalized_violation * normalized_violation;
            }
        }
    }

    return violation;
}

double ExcitationTrajectory::traj_score(const FourierTrajectory& traj, double dt, int identifiable_param_num) // 求激励轨迹的得分
{
    const int sample_cnt = static_cast<int>(traj.period() / dt);
    for (int i = 0; i < sample_cnt; i++) {
        q   = traj.position(i * dt);
        dq  = traj.velocity(i * dt);
        ddq = traj.acceleration(i * dt);

        const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
        if (i == 0) {
            Y.resize(sample_cnt * Yi.rows(), Yi.cols());
        }
        Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
    }
    // 进行SVD，对轨迹质量进行评分
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinV);
    S = svd.singularValues();

    double ret                = 0.0f;
    const int score_param_num = std::min<int>(identifiable_param_num, S.size());
    if (score_param_num <= 0) {
        return -std::numeric_limits<double>::infinity();
    }
    for (int i = 0; i < score_param_num; i++)
        ret = ret + std::log10(std::max(S[i], SVD_ZERO));
    return ret / score_param_num;
}
