#pragma once


#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

namespace arm_controller {

class ArmController : public controller_interface::ControllerInterface {
public:
    ArmController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    rclcpp::Time last_target_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace arm_controller
