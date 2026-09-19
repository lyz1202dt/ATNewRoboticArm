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
#include <robot_msgs/msg/arm_command.hpp>

constexpr std::size_t kMaxArmTrajectoryPoints  = 20;
constexpr std::size_t kMaxTaskDimensions       = 6;
constexpr std::size_t kArmCommandQueueCapacity = 64;
constexpr std::size_t kMaxRealtimeStateLength  = 15;

struct ArmCommandPointBuffer {
    std::array<double, kMaxTaskDimensions> pos{};
    std::array<double, kMaxTaskDimensions> vel{};
    std::array<double, kMaxTaskDimensions> acc{};
    std::uint8_t pos_size{0};
    std::uint8_t vel_size{0};
    std::uint8_t acc_size{0};
};

struct ArmCommandBuffer {
    std::array<char, 32> exp_state{};
    std::array<double, kMaxArmTrajectoryPoints> seconds{};
    std::array<ArmCommandPointBuffer, kMaxArmTrajectoryPoints> points{};
    std::size_t point_count{0};
    std::size_t seconds_count{0};

    bool has_state(const char* state) const {
        return state != nullptr && std::strncmp(exp_state.data(), state, exp_state.size()) == 0;
    }
};

class ArmCommandRingBuffer {
public:
    bool push(const robot_msgs::msg::ArmCommand& message) {
        const std::size_t write_index = write_index_.load(std::memory_order_relaxed);
        const std::size_t next_index  = next(write_index);
        if (next_index == read_index_.load(std::memory_order_acquire)) {
            return false;
        }
        if (message.exp_state.size() > kMaxRealtimeStateLength || message.points.size() > kMaxArmTrajectoryPoints
            || message.seconds.size() > kMaxArmTrajectoryPoints || message.points.size() != message.seconds.size()) {
            return false;
        }

        for (const auto& point : message.points) {
            if (point.pos.size() > kMaxTaskDimensions || point.vel.size() > kMaxTaskDimensions
                || point.acc.size() > kMaxTaskDimensions) {
                return false;
            }
            for (const double value : point.pos) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            for (const double value : point.vel) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            for (const double value : point.acc) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
        }
        for (const double value : message.seconds) {
            if (!std::isfinite(value) || value < 0.0) {
                return false;
            }
        }

        ArmCommandBuffer& slot = buffers_[write_index];
        slot.exp_state.fill('\0');
        std::memcpy(slot.exp_state.data(), message.exp_state.data(), message.exp_state.size());
        slot.point_count   = message.points.size();
        slot.seconds_count = message.seconds.size();
        for (std::size_t i = 0; i < slot.seconds_count; ++i) {
            slot.seconds[i] = message.seconds[i];
        }
        for (std::size_t i = 0; i < slot.point_count; ++i) {
            const auto& source = message.points[i];
            auto& destination  = slot.points[i];
            destination.pos_size = static_cast<std::uint8_t>(source.pos.size());
            destination.vel_size = static_cast<std::uint8_t>(source.vel.size());
            destination.acc_size = static_cast<std::uint8_t>(source.acc.size());
            for (std::size_t j = 0; j < source.pos.size(); ++j) {
                destination.pos[j] = source.pos[j];
            }
            for (std::size_t j = 0; j < source.vel.size(); ++j) {
                destination.vel[j] = source.vel[j];
            }
            for (std::size_t j = 0; j < source.acc.size(); ++j) {
                destination.acc[j] = source.acc[j];
            }
        }

        write_index_.store(next_index, std::memory_order_release);
        return true;
    }

    const ArmCommandBuffer* front() const {
        const std::size_t read_index = read_index_.load(std::memory_order_relaxed);
        if (read_index == write_index_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return &buffers_[read_index];
    }

    void pop() {
        const std::size_t read_index = read_index_.load(std::memory_order_relaxed);
        if (read_index != write_index_.load(std::memory_order_acquire)) {
            read_index_.store(next(read_index), std::memory_order_release);
        }
    }

    bool empty() const {
        return front() == nullptr;
    }

private:
    static constexpr std::size_t next(std::size_t index) {
        return (index + 1U) % kArmCommandQueueCapacity;
    }

    std::array<ArmCommandBuffer, kArmCommandQueueCapacity> buffers_{};
    std::atomic<std::size_t> read_index_{0};
    std::atomic<std::size_t> write_index_{0};
};

class FSMArmControlFactory : public FSMFactory {
public:
    FSMArmControlFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        node_ = node;
        model_=std::make_shared<ModelFromURDF>(node->get_parameter("urdf_path").as_string(),"link6");
        task_map_=std::make_shared<Default6DofTaskSpaceMapping>();
        arm_solve_=std::make_shared<ArmSolve>(model_,task_map_);    //加载机器人模型

        command_subscription_ = node_->create_subscription<robot_msgs::msg::ArmCommand>(
            "arm_traj", rclcpp::QoS(10), [this](const robot_msgs::msg::ArmCommand& msg) {
                command_buffer_.push(msg);
            });

        register_fsm(std::make_unique<IDELState>("idel", this));                  // 机械臂锁定在当前位置
        register_fsm(std::make_unique<ResetState>("reset", this));                // 机械臂复位
        register_fsm(std::make_unique<CartTrajState>("cart_traj", this));         // 执行笛卡尔轨迹
        register_fsm(std::make_unique<JointTrajState>("joint_traj", this));       //执行关节空间轨迹
        register_fsm(std::make_unique<ServoState>("servo", this));                //伺服动作，接收速度指令，将指令积分作为期望位置
        register_fsm(std::make_unique<AdmittanceState>("admittance", this));      //导纳控制
        register_fsm(std::make_unique<TeachPendantState>("teach_pendant", this));   //示教器，可外力拖动，可配置带阻尼，重力补偿
        register_fsm(std::make_unique<ParamterMeasureState>("measure", this));    //系统参数辨识

        set_init_state("idel");
    }

    std::string exp_state_name{"idel"};
    std::shared_ptr<ArmSolve> arm_solve_;
    std::shared_ptr<ModelBase> model_;
    std::shared_ptr<TaskMapping> task_map_;
    std::vector<robot_msgs::msg::MotorState> state_;
    std::vector<robot_msgs::msg::MotorCmd> command_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    ArmCommandRingBuffer command_buffer_;

private:
    rclcpp::Subscription<robot_msgs::msg::ArmCommand>::SharedPtr command_subscription_;
};
