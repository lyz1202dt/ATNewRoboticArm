#include "arm_fsm.hpp"

#include "arm_fsm_factory.hpp"

IDELState::IDELState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
}

bool IDELState::enter(const std::string& last_state) {
    (void)last_state;

    if (factory == nullptr || factory->state_.size() != factory->command_.size()) {
        return false;
    }

    hold_position_.resize(factory->state_.size());
    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        hold_position_[i] = factory->state_[i].position;
        apply_hold_command(i);
    }

    return true;
}

bool IDELState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string IDELState::check_switch() const {
    if (factory == nullptr || factory->exp_state_name.empty()) {    //允许切换到任何状态
        return fsm_name_;
    }

    return factory->exp_state_name;
}

bool IDELState::run() {
    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        auto& command    = factory->command_[i];
    command.position = hold_position_[i];
    command.velocity = 0.0f;
    command.torque   = 0.0f;
    command.kp       = i < factory->default_kp_.size() ? factory->default_kp_[i] : command.kp;
    command.kd       = i < factory->default_kd_.size() ? factory->default_kd_[i] : command.kd;
    command.ki       = 0.0f;
    }

    return true;
}

ResetState::ResetState(const std::string& name, std::any ctx)
    : FSM(name, ctx) {
}

bool ResetState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool ResetState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string ResetState::check_switch() const {
    return fsm_name_;
}

bool ResetState::run() {
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
    : FSM(name, ctx) {
}

bool TeachPendantState::enter(const std::string& last_state) {
    (void)last_state;
    return true;
}

bool TeachPendantState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

std::string TeachPendantState::check_switch() const {
    return fsm_name_;
}

bool TeachPendantState::run() {
    return true;
}
