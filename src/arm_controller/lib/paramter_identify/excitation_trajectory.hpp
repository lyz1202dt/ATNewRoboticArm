#pragma once

#include <atomic>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <Eigen/Dense>

class ExcitationTrajectory{
public:
    explicit ExcitationTrajectory(const std::string &urdf_path);
    void generate(rclcpp::Duration trajectory_time,int repeat_cnt=1,float joint_vel_limit_scale=0.8,float joint_pos_limit_scale=0.8);
    bool generate_is_finished();

    bool get_target_position(rclcpp::Duration time,Eigen::VectorXd &pos);
private:
    std::atomic_bool trajectory_generatefinished{false};
    void calc_traj();
};
