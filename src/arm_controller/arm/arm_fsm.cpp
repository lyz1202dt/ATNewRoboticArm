#include "arm_fsm.hpp"

#include "arm_fsm_factory.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include <rclcpp/rclcpp.hpp>

IDELState::IDELState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity = std::max({joint_count_, default_kp_param_.size(), default_kd_param_.size()});
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    hold_position_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
}

bool IDELState::enter(const std::string& last_state) {
    (void)last_state;

    if (factory == nullptr || factory->node_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
        default_kp_[i]    = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : command.kp;
        default_kd_[i]    = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : command.kd;
        command.position  = hold_position_[i];
        command.velocity  = 0.0f;
        command.torque    = 0.0f;
        command.kp        = default_kp_[i];
        command.kd        = default_kd_[i];
        command.ki        = 0.0f;   //只要i的值改变，底层会立即清空积分项
    }

    return true;
}

bool IDELState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string IDELState::check_switch() const {
    if (factory == nullptr || factory->exp_state_name.empty()) { // 允许切换到任何状态
        return fsm_name_;
    }

    return factory->exp_state_name;
}

bool IDELState::run() {
    if (factory == nullptr || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        auto& command    = factory->command_[i];
        command.position = hold_position_[i];
        command.velocity = 0.0f;
        command.torque   = 0.0f;
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    return true;
}

ResetState::ResetState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("reset_joint_pos", reset_joint_pos_param_);
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity =
        std::max({joint_count_, reset_joint_pos_param_.size(), default_kp_param_.size(), default_kd_param_.size()});
    reset_joint_pos_param_.reserve(param_capacity);
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    start_joint_pos_.resize(joint_count_);
    reset_joint_pos_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
}

bool ResetState::enter(const std::string& last_state) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    double reset_duration  = 3.0;
    double reset_tolerance = 0.01;
    factory->node_->get_parameter("reset_joint_pos", reset_joint_pos_param_);
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);
    factory->node_->get_parameter("reset_duration", reset_duration);
    factory->node_->get_parameter("reset_tolerance", reset_tolerance);

    for (std::size_t i = 0; i < start_joint_pos_.size(); ++i) {
        start_joint_pos_[i] = factory->state_[i].position;
        reset_joint_pos_[i] =
            i < reset_joint_pos_param_.size() ? static_cast<float>(reset_joint_pos_param_[i]) : 0.0f;
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : factory->command_[i].kp;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : factory->command_[i].kd;
    }

    reset_duration_   = reset_duration > 0.0 ? static_cast<float>(reset_duration) : 1.0f;
    reset_tolerance_  = reset_tolerance >= 0.0 ? static_cast<float>(reset_tolerance) : 0.01f;
    reset_start_time_ = factory->node_->now();
    progress_         = 0.0f;
    reset_done_       = joint_count_ == 0;
    return true;
}

bool ResetState::exit(const std::string& next_state) {
    return next_state == "idel";
}

std::string ResetState::check_switch() const {
    if (reset_done_) {
        return "idel";
    }
    return fsm_name_;
}

bool ResetState::run() {
    if (factory == nullptr || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    const float elapsed_s = static_cast<float>((factory->node_->now() - reset_start_time_).seconds());
    progress_             = std::min(1.0f, elapsed_s / reset_duration_);

    bool joint_reached = true;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        const float delta    = reset_joint_pos_[i] - start_joint_pos_[i];
        auto& command        = factory->command_[i];
        command.position     = start_joint_pos_[i] + delta * progress_;
        command.velocity     = progress_ < 1.0f ? delta / reset_duration_ : 0.0f;
        command.torque       = 0.0f;
        command.kp           = default_kp_[i];
        command.kd           = default_kd_[i];
        command.ki           = 0.0f;
        const float position_error = std::fabs(factory->state_[i].position - reset_joint_pos_[i]);
        joint_reached              = joint_reached && position_error <= reset_tolerance_;
    }

    reset_done_ = progress_ >= 1.0f && joint_reached;
    return true;
}

CartTrajState::CartTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool CartTrajState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool CartTrajState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string CartTrajState::check_switch() const {
    return fsm_name_;
}

bool CartTrajState::run() {
    return true;
}

JointTrajState::JointTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool JointTrajState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool JointTrajState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string JointTrajState::check_switch() const {
    return fsm_name_;
}

bool JointTrajState::run() {
    return true;
}

ServoState::ServoState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool ServoState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool ServoState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string ServoState::check_switch() const {
    return fsm_name_;
}

bool ServoState::run() {
    return true;
}

AdmittanceState::AdmittanceState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool AdmittanceState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string AdmittanceState::check_switch() const {
    return fsm_name_;
}

bool AdmittanceState::run() {
    return true;
}

TeachPendantState::TeachPendantState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity = std::max(joint_count_, default_kd_param_.size());
    default_kd_param_.reserve(param_capacity);
    default_kd_.resize(joint_count_);
}

bool TeachPendantState::enter(const std::string& last_state) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    factory->node_->get_parameter("default_kd", default_kd_param_);
    for (std::size_t i = 0; i < default_kd_.size(); ++i) {
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : factory->command_[i].kd;
    }
    return true;
}

bool TeachPendantState::exit(const std::string& next_state) {
    return next_state == "idel";
}

std::string TeachPendantState::check_switch() const {
    if (factory != nullptr && factory->exp_state_name == "idel") {
        return "idel";
    }
    return fsm_name_;
}

bool TeachPendantState::run() {
    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr
        || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    Eigen::VectorXd joint_pos(joint_count_);
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    Eigen::VectorXd gravity_torque;
    Eigen::VectorXd end_effector_pose;
    try {
        const Eigen::VectorXd task_zero = Eigen::VectorXd::Zero(6);
        gravity_torque = factory->arm_solve_->inverse_dynamic(joint_pos, task_zero, task_zero, task_zero);
        end_effector_pose = factory->arm_solve_->forward_kinamic(joint_pos);
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(factory->node_->get_logger(),
                             *factory->node_->get_clock(),
                             1000,
                             "Teach pendant gravity compensation failed: %s",
                             error.what());
        return false;
    }

    if (gravity_torque.size() != static_cast<Eigen::Index>(joint_count_) || !gravity_torque.allFinite()
        || end_effector_pose.size() != 6 || !end_effector_pose.allFinite()) {
        RCLCPP_WARN_THROTTLE(factory->node_->get_logger(),
                             *factory->node_->get_clock(),
                             1000,
                             "Teach pendant solve returned invalid result: gravity torque size=%ld, pose size=%ld",
                             gravity_torque.size(),
                             end_effector_pose.size());
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        auto& command    = factory->command_[i];
        command.position = factory->state_[i].position;
        command.velocity = 0.0f;
        command.torque   = static_cast<float>(gravity_torque(static_cast<Eigen::Index>(i)));
        command.kp       = 0.0f;
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    RCLCPP_INFO_THROTTLE(factory->node_->get_logger(),
                         *factory->node_->get_clock(),
                         200,
                         "Teach pendant end-effector pose: position[x=%.6f, y=%.6f, z=%.6f], "
                         "rotation_vector[rx=%.6f, ry=%.6f, rz=%.6f]",
                         end_effector_pose(0),
                         end_effector_pose(1),
                         end_effector_pose(2),
                         end_effector_pose(3),
                         end_effector_pose(4),
                         end_effector_pose(5));

    return true;
}
