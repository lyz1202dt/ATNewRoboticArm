#pragma once

#include "fsm.hpp"

#include <any>
#include <cstddef>
#include <string>
#include <vector>

class FSMArmControlFactory;


class IDELState : public FSM {
public:
    IDELState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;

private:
    void apply_hold_command(std::size_t index);

    FSMArmControlFactory* factory{nullptr};
    std::vector<float> hold_position_;
};

class ResetState : public FSM {
public:
    ResetState(const std::string& name, std::any ctx);

    bool enter(const std::string& last_state) override;
    bool exit(const std::string& next_state) override;
    std::string check_switch() const override;
    bool run() override;
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
};
