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
#include <vector>
#include <random>
#include "fourier_trajectory.hpp"

class ExcitationTrajectory{
public:
    explicit ExcitationTrajectory(const std::string &urdf_path);
    void generate(rclcpp::Duration trajectory_time,int repeat_cnt=1,float joint_vel_limit_scale=0.8,float joint_pos_limit_scale=0.8);
    bool generate_is_finished();

    bool get_target_position(rclcpp::Duration time,Eigen::VectorXd &pos);
private:
    std::atomic_bool trajectory_generatefinished{false};
    void calc_traj();
    bool traj_is_available(const FourierTrajectory& traj,double dt);

    //奇异值条件数/总体信息量
    std::tuple<double,double> traj_score(const FourierTrajectory& traj,double dt,int identifiable_param_num);

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
};
