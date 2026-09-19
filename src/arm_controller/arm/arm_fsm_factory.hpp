#pragma once

#include "arm.hpp"
#include "arm_fsm.hpp"
#include "default6dof_task.hpp"
#include "fsm_factory.hpp"
#include "model_from_urdf.hpp"
#include "modelbase.hpp"
#include "task.hpp"

#include <memory>
#include <rclcpp/qos.hpp>
#include <robot_msgs/msg/detail/joint_traj_cmd__struct.hpp>
#include <string>
#include <vector>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_msgs/msg/motor_cmd.hpp>
#include <robot_msgs/msg/motor_state.hpp>



class FSMArmControlFactory : public FSMFactory {
public:
    FSMArmControlFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        node_      = node;
        model_     = std::make_shared<ModelFromURDF>(node->get_parameter("urdf_path").as_string(), "link6");
        task_map_  = std::make_shared<Default6DofTaskSpaceMapping>();
        arm_solve_ = std::make_shared<ArmSolve>(model_, task_map_);         // 加载机器人模型


        register_fsm(std::make_unique<IDELState>("idel", this));            // 机械臂锁定在当前位置
        register_fsm(std::make_unique<ResetState>("reset", this));          // 机械臂复位
        register_fsm(std::make_unique<CartTrajState>("cart_traj", this));   // 执行笛卡尔轨迹
        register_fsm(std::make_unique<JointTrajState>("joint_traj", this)); // 执行关节空间轨迹
        register_fsm(std::make_unique<ServoState>("servo", this)); // 伺服动作，接收速度指令，将指令积分作为期望位置
        register_fsm(std::make_unique<AdmittanceState>("admittance", this)); // 导纳控制
        register_fsm(std::make_unique<TeachPendantState>("teach_pendant", this)); // 示教器，可外力拖动，可配置带阻尼，重力补偿
        register_fsm(std::make_unique<ParamterMeasureState>("measure", this)); // 系统参数辨识

        set_init_state("idel");
    }

    std::string exp_state_name{"idel"};
    std::shared_ptr<ArmSolve> arm_solve_;
    std::shared_ptr<ModelBase> model_;
    std::shared_ptr<TaskMapping> task_map_;
    std::vector<robot_msgs::msg::MotorState> state_;
    std::vector<robot_msgs::msg::MotorCmd> command_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

private:
};
