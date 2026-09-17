#pragma once

#include <atomic>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <Eigen/Dense>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <memory>
#include "fourier_trajectory.hpp"

class ExcitationTrajectory{
public:
    explicit ExcitationTrajectory(const std::string &urdf_path);
    void generate(double period, int repeat_cnt);
    bool generate_is_finished();

    bool get_target_position(const rclcpp::Duration &time,Eigen::VectorXd &pos);
private:
    std::atomic_bool trajectory_generatefinished{false};
    void calc_traj(double peroid);

    //评估轨迹可行性和质量
    bool traj_is_available(const FourierTrajectory& traj,double dt);
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

    std::shared_ptr<FourierTrajectory> best_traj;
    int traj_repeat_cnt{1};


    //计算线程
    std::shared_ptr<std::thread> calc_thread;
};
