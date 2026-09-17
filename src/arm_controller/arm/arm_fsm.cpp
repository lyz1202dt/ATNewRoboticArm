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
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
}

bool IDELState::enter(const std::string& last_state) {
    (void)last_state;

    if (factory == nullptr || factory->node_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
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
    double reset_duration_param  = 3.0;
    double reset_tolerance_param = 0.01;
    factory->node_->get_parameter("reset_duration", reset_duration_param);
    factory->node_->get_parameter("reset_tolerance", reset_tolerance_param);
    reset_duration_  = reset_duration_param > 0.0 ? static_cast<float>(reset_duration_param) : 1.0f;
    reset_tolerance_ = reset_tolerance_param >= 0.0 ? static_cast<float>(reset_tolerance_param) : 0.01f;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        reset_joint_pos_[i] = i < reset_joint_pos_param_.size() ? static_cast<float>(reset_joint_pos_param_[i]) : 0.0f;
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
}

bool ResetState::enter(const std::string& last_state) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < start_joint_pos_.size(); ++i) {
        start_joint_pos_[i] = factory->state_[i].position;
    }

    reset_start_time_ = factory->node_->now();
    progress_         = 0.0f;
    reset_done_       = joint_count_ == 0;
    return true;
}

bool ResetState::exit(const std::string& next_state) {
    if (factory != nullptr && next_state == "idel") {
        factory->exp_state_name = "idel";
    }
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
    joint_pos_.resize(joint_count_);
    task_zero_.setZero(6);
    gravity_torque_.resize(joint_count_);
    end_effector_pose_.resize(6);
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
}

bool TeachPendantState::enter(const std::string& last_state) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
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

    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_dynamic(joint_pos_, task_zero_, task_zero_, task_zero_, &gravity_torque_)
            || !factory->arm_solve_->forward_kinamic(joint_pos_, &end_effector_pose_)) {
            RCLCPP_WARN_THROTTLE(factory->node_->get_logger(),
                                 *factory->node_->get_clock(),
                                 1000,
                                 "Teach pendant solve failed");
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(factory->node_->get_logger(),
                             *factory->node_->get_clock(),
                             1000,
                             "Teach pendant gravity compensation failed: %s",
                             error.what());
        return false;
    }

    if (gravity_torque_.size() != static_cast<Eigen::Index>(joint_count_) || !gravity_torque_.allFinite()
        || end_effector_pose_.size() != 6 || !end_effector_pose_.allFinite()) {
        RCLCPP_WARN_THROTTLE(factory->node_->get_logger(),
                             *factory->node_->get_clock(),
                             1000,
                             "Teach pendant solve returned invalid result: gravity torque size=%ld, pose size=%ld",
                             gravity_torque_.size(),
                             end_effector_pose_.size());
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        auto& command    = factory->command_[i];
        command.position = factory->state_[i].position;
        command.velocity = 0.0f;
        command.torque   = static_cast<float>(gravity_torque_(static_cast<Eigen::Index>(i)));
        command.kp       = 0.0f;
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    RCLCPP_INFO_THROTTLE(factory->node_->get_logger(),
                         *factory->node_->get_clock(),
                         200,
                         "Teach pendant end-effector pose: position[x=%.6f, y=%.6f, z=%.6f], "
                         "rotation_vector[rx=%.6f, ry=%.6f, rz=%.6f]",
                         end_effector_pose_(0),
                         end_effector_pose_(1),
                         end_effector_pose_(2),
                         end_effector_pose_(3),
                         end_effector_pose_(4),
                         end_effector_pose_(5));

    return true;
}



ParamterMeasureState::ParamterMeasureState(const std::string& name, std::any ctx)
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
    factory->node_->get_parameter("measure_trajectory_period", trajectory_period_);
    factory->node_->get_parameter("measure_trajectory_repeat_cnt", trajectory_repeat_cnt_);

    const std::size_t param_capacity = std::max({joint_count_, default_kp_param_.size(), default_kd_param_.size()});
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    hold_position_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
    target_position_.resize(joint_count_);

    trajectory_period_     = trajectory_period_ > 0.0 ? trajectory_period_ : 10.0;
    trajectory_repeat_cnt_ = trajectory_repeat_cnt_ > 0 ? trajectory_repeat_cnt_ : 1;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    try {
        excitation_trajectory_ = std::make_shared<ExcitationTrajectory>(factory->node_->get_parameter("urdf_path").as_string());
        excitation_trajectory_->generate(trajectory_period_, trajectory_repeat_cnt_);
        trajectory_ready_ = excitation_trajectory_->generate_is_finished();
    } catch (const std::exception& error) {
        RCLCPP_WARN(factory->node_->get_logger(), "Failed to generate excitation trajectory: %s", error.what());
        excitation_trajectory_.reset();
        trajectory_ready_ = false;
    }
}

bool ParamterMeasureState::enter(const std::string& last_state) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (excitation_trajectory_ == nullptr || !trajectory_ready_) {
        return false;
    }

    tracking_started_ = false;
    measure_done_     = false;

    for (std::size_t i = 0; i < joint_count_; ++i) {
        hold_position_[i] = factory->state_[i].position;
    }

    return true;
}

bool ParamterMeasureState::exit(const std::string& next_state) {
    return next_state == "idel";
}

std::string ParamterMeasureState::check_switch() const {
    if (factory != nullptr && (measure_done_ || factory->exp_state_name == "idel")) {
        return "idel";
    }
    return fsm_name_;
}

bool ParamterMeasureState::run() {
    if (factory == nullptr || factory->node_ == nullptr || excitation_trajectory_ == nullptr
        || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    if (!tracking_started_) {
        measure_start_time_ = factory->node_->now();
        tracking_started_   = true;
    }

    const rclcpp::Duration elapsed = factory->node_->now() - measure_start_time_;
    const double total_duration    = trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_);
    if (elapsed.seconds() >= total_duration) {
        measure_done_ = true;
    }

    if (!excitation_trajectory_->get_target_position(elapsed, target_position_)
        || target_position_.size() != static_cast<Eigen::Index>(joint_count_) || !target_position_.allFinite()) {
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        auto& command    = factory->command_[i];
        command.position = static_cast<float>(target_position_(static_cast<Eigen::Index>(i)));
        command.velocity = 0.0f;
        command.torque   = 0.0f;
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    return true;
}
