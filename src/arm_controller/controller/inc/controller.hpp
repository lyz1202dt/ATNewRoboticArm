#pragma once


#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <../arm/arm_fsm.hpp>
#include <string>
#include <unordered_map>
#include <controller_interface/controller_interface_base.hpp>

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
    std::string command_interface_name(const std::string& joint_name, const std::string& interface_name) const;

    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::shared_ptr<FSMArmControlFactory> fsm_factory;
    std::vector<std::string> joints_name;
    std::vector<float> default_kp,default_kd;
};

}  // namespace arm_controller
