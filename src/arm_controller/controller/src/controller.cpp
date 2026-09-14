#include "../inc/controller.hpp"

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

namespace arm_controller {

ArmController::ArmController() {
}

controller_interface::CallbackReturn ArmController::on_init() {
    auto node   = get_node();
    fsm_factory = std::make_shared<FSMArmControlFactory>(node);

    auto_declare<std::vector<double>>("default_kp", {});
    auto_declare<std::vector<double>>("default_kd", {});
    auto_declare<std::string>("urdf_path", "");
    auto_declare<std::string>("exp_state", "idel");
    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            auto name = param.get_name();
            if (name == "exp_state") {
                fsm_factory->exp_state_name = param.as_string();
            } else {
            }
        }
        return result;
    });

    std::string urdf_path;
    node->get_parameter<std::string>("urdf_path", urdf_path);

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    joints_name.reserve(model.names.size() > 0 ? model.names.size() - 1 : 0);
    for (std::size_t i = 1; i < model.names.size(); ++i) {
        joints_name.push_back(model.names[i]);
    }

    // 加载默认kp和kd参数
    std::vector<double> default_kp_param;
    std::vector<double> default_kd_param;
    node->get_parameter("default_kp", default_kp_param);
    node->get_parameter("default_kd", default_kd_param);
    default_kp.assign(joints_name.size(), 0.0f);
    default_kd.assign(joints_name.size(), 0.0f);
    for (std::size_t i = 0; i < std::min(default_kp.size(), default_kp_param.size()); ++i) {
        default_kp[i] = static_cast<float>(default_kp_param[i]);
    }
    for (std::size_t i = 0; i < std::min(default_kd.size(), default_kd_param.size()); ++i) {
        default_kd[i] = static_cast<float>(default_kd_param[i]);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    fsm_factory->state_.resize(joints_name.size());
    fsm_factory->command_.resize(joints_name.size());
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    if (state_interfaces_.size() == joints_name.size() * 3 && fsm_factory->command_.size() == joints_name.size()) {
        for (std::size_t i = 0; i < joints_name.size(); ++i) {
            fsm_factory->command_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
            fsm_factory->command_[i].velocity = 0.0f;
            fsm_factory->command_[i].torque   = 0.0f;
            fsm_factory->command_[i].kp       = default_kp[i];
            fsm_factory->command_[i].kd       = default_kd[i];
            fsm_factory->command_[i].ki       = 0.0f;
        }
    }
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
        fsm_factory->state_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
        fsm_factory->state_[i].velocity = static_cast<float>(state_interfaces_[i * 3 + 1].get_value());
        fsm_factory->state_[i].torque   = static_cast<float>(state_interfaces_[i * 3 + 2].get_value());
    }

    bool ret = fsm_factory->run();
    if (!ret) {
        for (std::size_t i = 0; i < joints_name.size(); i++) { // 安全保护
            command_interfaces_[i * 6 + 0].set_value(state_interfaces_[i * 3 + 0].get_value());
            command_interfaces_[i * 6 + 1].set_value(0.0);
            command_interfaces_[i * 6 + 2].set_value(0.0);
            command_interfaces_[i * 6 + 3].set_value(default_kp[i]);
            command_interfaces_[i * 6 + 4].set_value(default_kd[i]);
            command_interfaces_[i * 6 + 5].set_value(0.0f);
        }
        return controller_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
        command_interfaces_[i * 6 + 0].set_value(fsm_factory->command_[i].position);
        command_interfaces_[i * 6 + 1].set_value(fsm_factory->command_[i].velocity);
        command_interfaces_[i * 6 + 2].set_value(fsm_factory->command_[i].torque);
        command_interfaces_[i * 6 + 3].set_value(fsm_factory->command_[i].kp);
        command_interfaces_[i * 6 + 4].set_value(fsm_factory->command_[i].kd);
        command_interfaces_[i * 6 + 5].set_value(fsm_factory->command_[i].ki);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
        cfg.names.push_back(name + "/kp");
        cfg.names.push_back(name + "/kd");
        cfg.names.push_back(name + "/ki");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

} // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
