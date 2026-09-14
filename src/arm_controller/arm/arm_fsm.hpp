#pragma once

#include "fsm.hpp"
#include "fsm_factory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_msgs/msg/detail/motor_cmd__struct.hpp>
#include <robot_msgs/msg/detail/motor_state__struct.hpp>
#include <robot_msgs/msg/motor_cmd.hpp>
#include <robot_msgs/msg/motor_state.hpp>
#include <thread>
#include <vector>


using namespace std::chrono_literals;

class FSMArmControlFactory;

class IDELState : public FSM {
public:
    IDELState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
        factory = std::any_cast<FSMArmControlFactory*>(ctx);
    }

    bool enter(const std::string& last_state) override {

        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
    FSMArmControlFactory* factory;
};

class ResetState : public FSM {
public:
    ResetState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};

class CartTrajState : public FSM {
public:
    CartTrajState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};

class JointTrajState : public FSM {
public:
    JointTrajState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};

class ServoState : public FSM {
public:
    ServoState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};

class AdmittanceState : public FSM {
public:
    AdmittanceState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};

class TeachPendantState : public FSM {
public:
    TeachPendantState(const std::string& name, std::any ctx)
        : FSM(name, ctx) {
    }

    bool enter(const std::string& last_state) override {
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const override {
        return fsm_name_;
    }

    bool run() override {
        return true;
    }
};


class FSMArmControlFactory : public FSMFactory {
public:
    FSMArmControlFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        node_ = node;
        register_fsm(new IDELState("idel", this));                  // 机械臂锁定在当前位置
        register_fsm(new ResetState("reset", this));                // 机械臂复位
        register_fsm(new CartTrajState("cart_traj", this));         // 执行笛卡尔轨迹
        register_fsm(new JointTrajState("joint_traj", this));       //执行关节空间轨迹
        register_fsm(new ServoState("servo", this));                //伺服动作，接收速度指令，将指令积分作为期望位置
        register_fsm(new AdmittanceState("admittance", this));      //导纳控制
        register_fsm(new AdmittanceState("teach_pendant", this));   //示教器，可外力拖动，可配置带阻尼，重力补偿
        
        set_init_state("idel");
    }
    std::string exp_state_name{"idel"};
    std::vector<robot_msgs::msg::MotorState> state_;
    std::vector<robot_msgs::msg::MotorCmd> command_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
