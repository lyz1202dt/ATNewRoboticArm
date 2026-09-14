#pragma once

#include "fsm.hpp"
#include "fsm_factory.hpp"
#include <robot_msgs/msg/detail/motor_state__struct.hpp>
#include <robot_msgs/msg/motor_cmd.hpp>
#include <robot_msgs/msg/motor_state.hpp>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>


using namespace std::chrono_literals;

class IDELState : public FSM{
public:
    IDELState(const std::string &name,FSMFactory* factory) : FSM(name,factory)
    {
        
    }

    bool enter(const std::string& last_state) override{
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const  override{
        return fsm_name_;
    }

    bool run()  override{
        return true;
    }
};

class ResetingState : public FSM{
public:
    ResetingState(const std::string &name,FSMFactory* factory) : FSM(name,factory)
    {
        
    }

    bool enter(const std::string& last_state) override{
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const  override{
        return fsm_name_;
    }

    bool run()  override{
        return true;
    }
};

class RunningState : public FSM{
public:
    RunningState(const std::string &name,FSMFactory* factory) : FSM(name,factory)
    {
        
    }

    bool enter(const std::string& last_state) override{
        (void)last_state;
        return true;
    }

    bool exit(const std::string& next_state) override {
        (void)next_state;
        return true;
    }

    std::string check_switch() const  override{
        return fsm_name_;
    }

    bool run()  override{
        return true;
    }
};


class FSMArmControlFactory : public FSMFactory {
public:
    FSMArmControlFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        node_=node;
        register_fsm(new IDELState("idel",this));       //关节输出不再更新，位控模式锁定当前位置，不再执行调度器，切出时重排列调度器
        register_fsm(new ResetingState("reseting",this));   //在状态机中插值到复位位置
        register_fsm(new RunningState("running",this));    //执行调度器
        set_init_state("idel");
    }
    ~FSMArmControlFactory(){
       
    }
    std::vector<robot_msgs::msg::MotorState> state;
    std::vector<robot_msgs::msg::MotorState> command;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};