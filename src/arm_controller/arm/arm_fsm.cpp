#include "arm_fsm.hpp"

#include "arm_fsm_factory.hpp"
#include "trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <robot_msgs/msg/arm_command.hpp>
#include <robot_msgs/msg/detail/arm_command__struct.hpp>

namespace {

// 单条轨迹的最大点数。load_trajectory_from_command 依赖各 FSM 的 traj 预分配容量不小于它，
// 以便在 run() 中复用预分配 buffer（clear + add_point），避免实时路径内重新分配内存。
constexpr int kMaxTrajectoryPoints = 20;

int model_dof_from_context(std::any ctx) {
    auto* factory = std::any_cast<FSMArmControlFactory*>(ctx);
    if (factory == nullptr || factory->model_ == nullptr) {
        return 0;
    }
    return factory->model_->dof();
}

bool fill_point_from_msg(const robot_msgs::msg::Point& msg, std::size_t dimension, Point* point) {
    if (point == nullptr || msg.pos.size() != dimension) {
        return false;
    }
    if ((!msg.vel.empty() && msg.vel.size() != dimension) || (!msg.acc.empty() && msg.acc.size() != dimension)) {
        return false;
    }

    for (std::size_t i = 0; i < dimension; ++i) {
        const auto index  = static_cast<Eigen::Index>(i);
        point->pos(index) = msg.pos[i];
        point->vel(index) = msg.vel.empty() ? 0.0 : msg.vel[i];
        point->acc(index) = msg.acc.empty() ? 0.0 : msg.acc[i];
    }

    return point->pos.allFinite() && point->vel.allFinite() && point->acc.allFinite();
}

bool load_trajectory_from_command(const robot_msgs::msg::ArmCommand& cmd, std::size_t dimension, Point* point, Trajectory* traj) {
    if (point == nullptr || traj == nullptr || dimension == 0 || cmd.points.empty() || cmd.points.size() != cmd.seconds.size()
        || cmd.points.size() > traj->capacity()) {
        return false;
    }

    traj->clear();
    for (std::size_t i = 0; i < cmd.points.size(); ++i) {
        if (cmd.seconds[i] < 0.0 || !std::isfinite(cmd.seconds[i]) || !fill_point_from_msg(cmd.points[i], dimension, point)) {
            return false;
        }
        if (!traj->add_point(*point, rclcpp::Duration::from_seconds(cmd.seconds[i]))) {
            return false;
        }
    }

    return traj->size() > 0;
}

void hold_current_joint_position(
    FSMArmControlFactory* factory, std::size_t joint_count, const std::vector<float>& default_kp, const std::vector<float>& default_kd) {
    if (factory == nullptr || factory->state_.size() != joint_count || factory->command_.size() != joint_count
        || default_kp.size() != joint_count || default_kd.size() != joint_count) {
        return;
    }

    for (std::size_t i = 0; i < joint_count; ++i) {
        auto& command    = factory->command_[i];
        command.position = factory->state_[i].position;
        command.velocity = 0.0f;
        command.torque   = 0.0f;
        command.kp       = default_kp[i];
        command.kd       = default_kd[i];
        command.ki       = 0.0f;
    }
}

void load_default_gains(
    FSMArmControlFactory* factory, std::size_t joint_count, std::vector<double>* default_kp_param, std::vector<double>* default_kd_param,
    std::vector<float>* default_kp, std::vector<float>* default_kd) {
    if (factory == nullptr || factory->node_ == nullptr || default_kp_param == nullptr || default_kd_param == nullptr
        || default_kp == nullptr || default_kd == nullptr) {
        return;
    }

    factory->node_->get_parameter("default_kp", *default_kp_param);
    factory->node_->get_parameter("default_kd", *default_kd_param);

    const std::size_t param_capacity = std::max({joint_count, default_kp_param->size(), default_kd_param->size()});
    default_kp_param->reserve(param_capacity);
    default_kd_param->reserve(param_capacity);
    default_kp->resize(joint_count);
    default_kd->resize(joint_count);
    for (std::size_t i = 0; i < joint_count; ++i) {
        (*default_kp)[i] = i < default_kp_param->size() ? static_cast<float>((*default_kp_param)[i]) : 0.0f;
        (*default_kd)[i] = i < default_kd_param->size() ? static_cast<float>((*default_kd_param)[i]) : 0.0f;
    }
}

bool trajectory_finished(const Trajectory& traj, const rclcpp::Time& start_time, const rclcpp::Time& time) {
    // points 是预分配的定长池，不能直接取 points.back()/points.empty()；
    // 必须通过 Trajectory 的 size()/back() 接口按实际已加入的点数（index）判断。
    return !traj.empty() && (time - start_time) >= traj.back().time;
}

} // namespace

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

bool IDELState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

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
        command.ki        = 0.0f;                                // 只要i的值改变，底层会立即清空积分项
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

bool IDELState::run(const rclcpp::Time& time) {
    (void)time;

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
        default_kp_[i]      = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i]      = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
}

bool ResetState::enter(const std::string& last_state, const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < start_joint_pos_.size(); ++i) {
        start_joint_pos_[i] = factory->state_[i].position;
    }

    reset_start_time_ = time;
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

bool ResetState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    const float elapsed_s = static_cast<float>((time - reset_start_time_).seconds());
    progress_             = std::min(1.0f, elapsed_s / reset_duration_);

    bool joint_reached = true;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        const float delta          = reset_joint_pos_[i] - start_joint_pos_[i];
        auto& command              = factory->command_[i];
        command.position           = start_joint_pos_[i] + delta * progress_;
        command.velocity           = progress_ < 1.0f ? delta / reset_duration_ : 0.0f;
        command.torque             = 0.0f;
        command.kp                 = default_kp_[i];
        command.kd                 = default_kd_[i];
        command.ki                 = 0.0f;
        const float position_error = std::fabs(factory->state_[i].position - reset_joint_pos_[i]);
        joint_reached              = joint_reached && position_error <= reset_tolerance_;
    }

    reset_done_ = progress_ >= 1.0f && joint_reached;
    return true;
}

CartTrajState::CartTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , point(6)
    , traj(6, kMaxTrajectoryPoints)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    joint_count_ = model_dof_from_context(ctx);
    task_dof_    = 6;
    joint_pos_.resize(static_cast<Eigen::Index>(joint_count_));
    torque_.resize(static_cast<Eigen::Index>(joint_count_));
    task_force_.setZero(static_cast<Eigen::Index>(task_dof_));
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);
}

bool CartTrajState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || default_kp_.size() != joint_count_ || default_kd_.size() != joint_count_) {
        return false;
    }

    state = TrajPhase::STOP;
    traj.stop();
    hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
    return true;
}

bool CartTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    return true;
}

std::string CartTrajState::check_switch() const {
    if (factory == nullptr) {
        return fsm_name_;
    }
    if (!factory->cmd_queue.empty()) {
        const std::string exp_state_name = std::get<1>(factory->cmd_queue.front()).exp_state;
        if (exp_state_name != fsm_name_) {
            return exp_state_name;
        }
    } else if (state == TrajPhase::STOP && !factory->exp_state_name.empty() && factory->exp_state_name != fsm_name_) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool CartTrajState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (state == TrajPhase::STOP) {
        if (factory->cmd_queue.empty() || std::get<1>(factory->cmd_queue.front()).exp_state != fsm_name_) {
            hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
            return true;
        }

        const auto& cmd = std::get<1>(factory->cmd_queue.front());
        if (!load_trajectory_from_command(cmd, task_dof_, &point, &traj)) {
            RCLCPP_WARN(factory->node_->get_logger(), "Invalid Cartesian trajectory command");
            return false;
        }
        traj_start_time_ = time;
        traj.start(time);
        state = TrajPhase::MOVING;
    }

    if (state == TrajPhase::MOVING) {
        traj.update(time, point);
        if (point.pos.size() != static_cast<Eigen::Index>(task_dof_) || point.vel.size() != static_cast<Eigen::Index>(task_dof_)
            || point.acc.size() != static_cast<Eigen::Index>(task_dof_) || !point.pos.allFinite() || !point.vel.allFinite()
            || !point.acc.allFinite()) {
            return false;
        }

        try {
            if (!factory->arm_solve_->inverse_kinamic(point.pos, &joint_pos_)
                || joint_pos_.size() != static_cast<Eigen::Index>(joint_count_) || !joint_pos_.allFinite()
                || !factory->arm_solve_->inverse_dynamic(joint_pos_, point.vel, point.acc, task_force_, &torque_)
                || torque_.size() != static_cast<Eigen::Index>(joint_count_) || !torque_.allFinite()) {
                return false;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Cartesian trajectory solve failed: %s", error.what());
            return false;
        }

        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index = static_cast<Eigen::Index>(i);
            auto& command    = factory->command_[i];
            command.position = static_cast<float>(joint_pos_(index));
            command.velocity = 0.0f;
            command.torque   = static_cast<float>(torque_(index));
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if (trajectory_finished(traj, traj_start_time_, time)) {
            traj.stop();
            if (!factory->cmd_queue.empty() && std::get<1>(factory->cmd_queue.front()).exp_state == fsm_name_) {
                factory->cmd_queue.pop();
            }
            state = TrajPhase::STOP;
        }
    }

    return true;
}

JointTrajState::JointTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , point(model_dof_from_context(ctx))
    , traj(model_dof_from_context(ctx), kMaxTrajectoryPoints)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    joint_count_ = model_dof_from_context(ctx);
    torque.resize(static_cast<Eigen::Index>(joint_count_));
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);
}

bool JointTrajState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->model_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || default_kp_.size() != joint_count_ || default_kd_.size() != joint_count_) {
        return false;
    }

    state = TrajPhase::STOP;
    traj.stop();
    hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
    return true;
}

bool JointTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    return true;
}

std::string JointTrajState::check_switch() const {
    if (factory == nullptr) {
        return fsm_name_;
    }
    if (!factory->cmd_queue.empty()) {
        const std::string exp_state_name = std::get<1>(factory->cmd_queue.front()).exp_state;
        if (exp_state_name != fsm_name_) {
            return exp_state_name;
        }
    } else if (state == TrajPhase::STOP && !factory->exp_state_name.empty() && factory->exp_state_name != fsm_name_) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool JointTrajState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || factory->model_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (state == TrajPhase::STOP) {
        if (factory->cmd_queue.empty() || std::get<1>(factory->cmd_queue.front()).exp_state != fsm_name_) {
            hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
            return true;
        }

        auto const& cmd = std::get<1>(factory->cmd_queue.front());
        if (!load_trajectory_from_command(cmd, joint_count_, &point, &traj)) {
            RCLCPP_WARN(factory->node_->get_logger(), "Invalid joint trajectory command");
            return false;
        }
        traj_start_time_ = time;
        state            = TrajPhase::MOVING;
        traj.start(time);
    }

    if (state == TrajPhase::MOVING) {
        traj.update(time, point);
        if (point.pos.size() != static_cast<Eigen::Index>(joint_count_) || point.vel.size() != static_cast<Eigen::Index>(joint_count_)
            || point.acc.size() != static_cast<Eigen::Index>(joint_count_) || !point.pos.allFinite() || !point.vel.allFinite()
            || !point.acc.allFinite()) {
            return false;
        }

        try {
            if (!factory->model_->inverse_dynamic(point.pos, point.vel, point.acc, &torque)
                || torque.size() != static_cast<Eigen::Index>(joint_count_) || !torque.allFinite()) {
                return false;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Joint trajectory inverse dynamics failed: %s",
                error.what());
            return false;
        }

        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index = static_cast<Eigen::Index>(i);
            auto& command    = factory->command_[i];
            command.position = static_cast<float>(point.pos(index));
            command.velocity = static_cast<float>(point.vel(index));
            command.torque   = static_cast<float>(torque(index));
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if (trajectory_finished(traj, traj_start_time_, time)) {
            traj.stop();
            if (!factory->cmd_queue.empty() && std::get<1>(factory->cmd_queue.front()).exp_state == fsm_name_) {
                factory->cmd_queue.pop();
            }
            state = TrajPhase::STOP;
        }
    }
    return true;
}

ServoState::ServoState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool ServoState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;
    return true;
}

bool ServoState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string ServoState::check_switch() const {
    return fsm_name_;
}

bool ServoState::run(const rclcpp::Time& time) {
    (void)time;
    return true;
}

AdmittanceState::AdmittanceState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool AdmittanceState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;
    return true;
}

bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string AdmittanceState::check_switch() const {
    return fsm_name_;
}

bool AdmittanceState::run(const rclcpp::Time& time) {
    (void)time;
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

bool TeachPendantState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)time;

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

bool TeachPendantState::run(const rclcpp::Time& time) {
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_dynamic(joint_pos_, task_zero_, task_zero_, task_zero_, &gravity_torque_)
            || !factory->arm_solve_->forward_kinamic(joint_pos_, &end_effector_pose_)) {
            RCLCPP_WARN_THROTTLE(factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Teach pendant solve failed");
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Teach pendant gravity compensation failed: %s",
            error.what());
        return false;
    }

    if (gravity_torque_.size() != static_cast<Eigen::Index>(joint_count_) || !gravity_torque_.allFinite() || end_effector_pose_.size() != 6
        || !end_effector_pose_.allFinite()) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000,
            "Teach pendant solve returned invalid result: gravity torque size=%ld, pose size=%ld", gravity_torque_.size(),
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

    RCLCPP_INFO_THROTTLE(
        factory->node_->get_logger(), *factory->node_->get_clock(), 200,
        "Teach pendant end-effector pose: position[x=%.6f, y=%.6f, z=%.6f], "
        "rotation_vector[rx=%.6f, ry=%.6f, rz=%.6f]",
        end_effector_pose_(0), end_effector_pose_(1), end_effector_pose_(2), end_effector_pose_(3), end_effector_pose_(4),
        end_effector_pose_(5));

    return true;
}



ParamterMeasureState::ParamterMeasureState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx))
    , move_to_start_trajectory_(factory->model_->dof(), 4) {
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
    factory->node_->get_parameter("measure_move_to_start_duration", move_to_start_duration_);
    factory->node_->get_parameter("measure_csv_file_path", measure_csv_file_path_);
    factory->node_->get_parameter("measure_record_sample_rate", measure_record_sample_rate_);
    double end_effector_x_lower_limit;
    double end_effector_x_upper_limit;
    double end_effector_y_lower_limit;
    double end_effector_y_upper_limit;
    double end_effector_z_lower_limit;
    double end_effector_z_upper_limit;
    factory->node_->get_parameter("measure_end_effector_x_lower_limit", end_effector_x_lower_limit);
    factory->node_->get_parameter("measure_end_effector_x_upper_limit", end_effector_x_upper_limit);
    factory->node_->get_parameter("measure_end_effector_y_lower_limit", end_effector_y_lower_limit);
    factory->node_->get_parameter("measure_end_effector_y_upper_limit", end_effector_y_upper_limit);
    factory->node_->get_parameter("measure_end_effector_z_lower_limit", end_effector_z_lower_limit);
    factory->node_->get_parameter("measure_end_effector_z_upper_limit", end_effector_z_upper_limit);

    const std::size_t param_capacity = std::max({joint_count_, default_kp_param_.size(), default_kd_param_.size()});
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    hold_position_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
    target_position_.resize(joint_count_);
    excitation_start_position_.resize(joint_count_);
    excitation_end_position_.resize(joint_count_);
    measured_position_.resize(joint_count_);
    measured_velocity_.resize(joint_count_);
    measured_torque_.resize(joint_count_);
    start_point_.pos.resize(joint_count_);
    start_point_.vel.resize(joint_count_);
    start_point_.acc.resize(joint_count_);
    end_point_.pos.resize(joint_count_);
    end_point_.vel.resize(joint_count_);
    end_point_.acc.resize(joint_count_);

    trajectory_period_          = trajectory_period_ > 0.0 ? trajectory_period_ : 10.0;
    trajectory_repeat_cnt_      = trajectory_repeat_cnt_ > 0 ? trajectory_repeat_cnt_ : 1;
    move_to_start_duration_     = move_to_start_duration_ > 0.0 ? move_to_start_duration_ : 3.0;
    measure_record_sample_rate_ = measure_record_sample_rate_ > 0.0 ? measure_record_sample_rate_ : 500.0;
    if (measure_csv_file_path_.empty()) {
        measure_csv_file_path_ = "/tmp/measured_for_identification.csv";
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    const std::string urdf_path                        = factory->node_->get_parameter("urdf_path").as_string();
    excitation_trajectory_                             = std::make_shared<ExcitationTrajectory>(urdf_path);
    excitation_trajectory_->end_effector_x_lower_limit = end_effector_x_lower_limit;
    excitation_trajectory_->end_effector_x_upper_limit = end_effector_x_upper_limit;
    excitation_trajectory_->end_effector_y_lower_limit = end_effector_y_lower_limit;
    excitation_trajectory_->end_effector_y_upper_limit = end_effector_y_upper_limit;
    excitation_trajectory_->end_effector_z_lower_limit = end_effector_z_lower_limit;
    excitation_trajectory_->end_effector_z_upper_limit = end_effector_z_upper_limit;
    paramter_identify_                                 = std::make_unique<ParamterIdentify>(urdf_path);
    const auto record_capacity =
        static_cast<int>(std::ceil(trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_) * measure_record_sample_rate_)) + 2;
    paramter_identify_->application_memory(record_capacity, static_cast<int>(joint_count_));
}

bool ParamterMeasureState::enter(const std::string& last_state, const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || excitation_trajectory_ == nullptr || paramter_identify_ == nullptr) {
        return false;
    }

    if (!paramter_identify_->reset_recording()) {
        RCLCPP_WARN(factory->node_->get_logger(), "Previous parameter identification CSV save is still running");
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
        command.position  = hold_position_[i];
        command.velocity  = 0.0f;
        command.torque    = 0.0f;
        command.kp        = default_kp_[i];
        command.kd        = default_kd_[i];
        command.ki        = 0.0f;
    }

    measure_done_                 = false;
    trajectory_generation_failed_ = false;
    csv_save_requested_           = false;
    measure_phase_                = MeasurePhase::WaitingForTrajectory;
    phase_start_time_             = time;
    measure_start_time_           = time;
    excitation_trajectory_->generate(trajectory_period_, trajectory_repeat_cnt_);

    return true;
}

bool ParamterMeasureState::exit(const std::string& next_state) {
    return next_state == "idel";
}

std::string ParamterMeasureState::check_switch() const {
    if (factory != nullptr && measure_phase_ == MeasurePhase::HoldingEnd && factory->exp_state_name == "idel") {
        return "idel";
    }
    return fsm_name_;
}

bool ParamterMeasureState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || excitation_trajectory_ == nullptr || paramter_identify_ == nullptr
        || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    if (measure_phase_ == MeasurePhase::WaitingForTrajectory) {
        for (std::size_t i = 0; i < joint_count_; ++i) {
            auto& command    = factory->command_[i];
            command.position = hold_position_[i];
            command.velocity = 0.0f;
            command.torque   = 0.0f;
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if (trajectory_generation_failed_ || excitation_trajectory_->generate_failed()) {
            return false;
        }
        if (!excitation_trajectory_->generate_is_finished()) {
            return true;
        }

        const auto zero_duration    = rclcpp::Duration::from_seconds(0.0);
        const double total_duration = trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_);
        if (!excitation_trajectory_->get_target_position(zero_duration, excitation_start_position_)
            || !excitation_trajectory_->get_target_position(rclcpp::Duration::from_seconds(total_duration), excitation_end_position_)
            || excitation_start_position_.size() != static_cast<Eigen::Index>(joint_count_)
            || excitation_end_position_.size() != static_cast<Eigen::Index>(joint_count_) || !excitation_start_position_.allFinite()
            || !excitation_end_position_.allFinite()) {
            return false;
        }

        start_point_.time = zero_duration;
        end_point_.time   = zero_duration;

        for (std::size_t i = 0; i < joint_count_; ++i) {
            start_point_.pos(static_cast<Eigen::Index>(i)) = hold_position_[i];
        }
        start_point_.vel.setZero();
        start_point_.acc.setZero();

        end_point_.pos = excitation_start_position_;
        end_point_.vel.setZero();
        end_point_.acc.setZero();

        move_to_start_trajectory_.add_point(start_point_, zero_duration);
        move_to_start_trajectory_.add_point(end_point_, rclcpp::Duration::from_seconds(move_to_start_duration_));
        move_to_start_trajectory_.start(time);
        phase_start_time_ = time;
        RCLCPP_INFO(factory->node_->get_logger(), "最小可辨识参数数量为:%d", excitation_trajectory_->get_available_param_num());
        measure_phase_ = MeasurePhase::MovingToStart;
    }

    if (measure_phase_ == MeasurePhase::MovingToStart) {
        move_to_start_trajectory_.update(time, move_to_start_point_);
        if (move_to_start_point_.pos.size() != static_cast<Eigen::Index>(joint_count_) || !move_to_start_point_.pos.allFinite()) {
            return false;
        }

        const bool has_velocity =
            move_to_start_point_.vel.size() == static_cast<Eigen::Index>(joint_count_) && move_to_start_point_.vel.allFinite();
        for (std::size_t i = 0; i < joint_count_; ++i) {
            auto& command    = factory->command_[i];
            command.position = static_cast<float>(move_to_start_point_.pos(static_cast<Eigen::Index>(i)));
            command.velocity = has_velocity ? static_cast<float>(move_to_start_point_.vel(static_cast<Eigen::Index>(i))) : 0.0f;
            command.torque   = 0.0f;
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if ((time - phase_start_time_).seconds() < move_to_start_duration_) {
            return true;
        }

        measure_start_time_ = time;
        measure_phase_      = MeasurePhase::ExecutingTrajectory;
        return true;
    }

    if (measure_phase_ == MeasurePhase::ExecutingTrajectory) {
        const rclcpp::Duration elapsed = time - measure_start_time_;
        const double total_duration    = trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_);
        const bool trajectory_finished = elapsed.seconds() >= total_duration;
        if (trajectory_finished) {
            target_position_ = excitation_end_position_;
        } else if (
            !excitation_trajectory_->get_target_position(elapsed, target_position_)
            || target_position_.size() != static_cast<Eigen::Index>(joint_count_) || !target_position_.allFinite()) {
            return false;
        }

        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index          = static_cast<Eigen::Index>(i);
            measured_position_(index) = factory->state_[i].position;
            measured_velocity_(index) = factory->state_[i].velocity;
            measured_torque_(index)   = factory->state_[i].torque;
        }
        paramter_identify_->record_value(measured_position_, measured_velocity_, measured_torque_);

        if (trajectory_finished) {
            measure_done_  = true;
            measure_phase_ = MeasurePhase::HoldingEnd;
            if (!csv_save_requested_) {
                csv_save_requested_ = paramter_identify_->save_csv(measure_csv_file_path_);
            }
        }
    } else if (measure_phase_ == MeasurePhase::HoldingEnd) {
        target_position_ = excitation_end_position_;
    }

    if (target_position_.size() != static_cast<Eigen::Index>(joint_count_) || !target_position_.allFinite()) {
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
