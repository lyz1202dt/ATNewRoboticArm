#pragma once

#include "fsm.hpp"

#include <any>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include "excitation_trajectory.hpp"

class FSMArmControlFactory;


class IDELState : public FSM {
public:
    IDELState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;

private:
    FSMArmControlFactory* factory{nullptr};
    std::size_t joint_count_{0};
    std::vector<float> hold_position_;
    std::vector<float> default_kp_;
    std::vector<float> default_kd_;
    std::vector<double> default_kp_param_;
    std::vector<double> default_kd_param_;
};

class ResetState : public FSM {
public:
    ResetState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
private:
    FSMArmControlFactory* factory{nullptr};
    std::size_t joint_count_{0};
    std::vector<float> start_joint_pos_;
    std::vector<float> reset_joint_pos_;
    std::vector<float> default_kp_;
    std::vector<float> default_kd_;
    std::vector<double> reset_joint_pos_param_;
    std::vector<double> default_kp_param_;
    std::vector<double> default_kd_param_;
    rclcpp::Time reset_start_time_;
    float reset_duration_{3.0f};
    float reset_tolerance_{0.01f};
    float progress_{0.0f};
    bool reset_done_{false};
};

class CartTrajState : public FSM {
public:
    CartTrajState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
};

class JointTrajState : public FSM {
public:
    JointTrajState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
};

class ServoState : public FSM {
public:
    ServoState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
};

class AdmittanceState : public FSM {
public:
    AdmittanceState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
};

class TeachPendantState : public FSM {
public:
    TeachPendantState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;

private:
    FSMArmControlFactory* factory{nullptr};
    std::size_t joint_count_{0};
    std::vector<float> default_kd_;
    std::vector<double> default_kd_param_;
    Eigen::VectorXd joint_pos_;
    Eigen::VectorXd task_zero_;
    Eigen::VectorXd gravity_torque_;
    Eigen::VectorXd end_effector_pose_;
};

class ParamterMeasureState : public FSM {
public:
    ParamterMeasureState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;

private:
    FSMArmControlFactory* factory{nullptr};
    std::size_t joint_count_{0};
    std::shared_ptr<ExcitationTrajectory> excitation_trajectory_;
    std::vector<float> hold_position_;
    std::vector<float> default_kp_;
    std::vector<float> default_kd_;
    std::vector<double> default_kp_param_;
    std::vector<double> default_kd_param_;
    Eigen::VectorXd target_position_;
    rclcpp::Time measure_start_time_;
    double trajectory_period_{10.0};
    int trajectory_repeat_cnt_{1};
    bool trajectory_ready_{false};
    bool tracking_started_{false};
    bool measure_done_{false};
};
