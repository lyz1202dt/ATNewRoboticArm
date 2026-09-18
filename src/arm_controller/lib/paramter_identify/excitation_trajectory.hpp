#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "fourier_trajectory.hpp"

class ExcitationTrajectory{
public:
    explicit ExcitationTrajectory(const std::string &urdf_path);
    ~ExcitationTrajectory();
    void generate(double period, int repeat_cnt);
    bool generate_is_finished();
    bool generate_failed() const;

    bool get_target_position(const rclcpp::Duration &time,Eigen::VectorXd &pos);

    int get_available_param_num() const;
    
private:
    std::atomic_bool trajectory_generatefinished{false};
    std::atomic_bool trajectory_generatefailed{false};

    //评估轨迹可行性和质量
    double traj_score(const FourierTrajectory& traj,double dt,int identifiable_param_num);
    double traj_constraint_violation(const FourierTrajectory& traj,double dt);

    pinocchio::Model model_;
    pinocchio::Data data_;


    //参数
    const double SVD_ZERO=1e-6;
    double svd_pos_lower{-2.0};
    double svd_pos_upper{2.0};
    double svd_vel_upper{4.0};
    double svd_acc_upper{6.0};
    int svd_Y_stack_num{30};    //堆叠30个Y矩阵评估矩阵特性

    double traj_pos_soft_limit_rate{0.7};    //关节软限位，默认70%

    //缓存
    Eigen::VectorXd q, dq, ddq;
    Eigen::MatrixXd Y;
    Eigen::VectorXd S;
    Eigen::MatrixXd best_coefficients_;
    Eigen::MatrixXd param_mat;

    int identifiable_parameter_num_{0};

    std::mutex start_calc_mtx_;
    std::condition_variable start_calc_cv_;
    bool start_calc_flag{false};
    std::atomic_bool exit_thread{false};
    void calc_func();

    std::shared_ptr<FourierTrajectory> best_traj;
    int traj_repeat_cnt{1};
    double period_s{10.0};

    std::thread calc_thread_;
};
