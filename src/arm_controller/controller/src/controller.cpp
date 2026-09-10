#include "../inc/controller.hpp"

#include <algorithm>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arm_controller {

ArmController::ArmController() = default;

controller_interface::CallbackReturn ArmController::on_init() {
    auto node = get_node();

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            bool handled_gain = false;
            
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    auto node = get_node();
    
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    last_target_log_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // for (const auto& name : joints_name_) {
    //     cfg.names.push_back(name + "/effort");
    // }
    return cfg;
}

controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // for (const auto& name : joints_name_) {
    //     cfg.names.push_back(name + "/position");
    //     cfg.names.push_back(name + "/velocity");
    //     cfg.names.push_back(name + "/effort");
    // }
    return cfg;
}

}  // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
