#include "../inc/controller.hpp"

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


namespace arm_controller {

ArmController::ArmController() {
}

controller_interface::CallbackReturn ArmController::on_init() {
    auto node   = get_node();
    fsm_factory = std::make_shared<FSMArmControlFactory>(node);

    node->declare_parameter<std::vector<float>>("default_kp");
    node->declare_parameter<std::vector<float>>("default_kd");
    node->declare_parameter<std::string>("urdf_path", "");
    node->declare_parameter<std::string>("exp_state", "idel");
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

    // 从URDF加载关节名字
    std::string urdf_path;
    node->get_parameter<std::string>("urdf_path", urdf_path);
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    joints_name.resize(joints_name.size());
    for (int i = 1; i < joints_name.size(); i++)
        joints_name[i - 1] = model.names[i];

    // 加载默认kp和kd参数
    default_kp.resize(joints_name.size());
    default_kd.resize(joints_name.size());
    node->get_parameter("default_kp", default_kp);
    node->get_parameter("default_kd", default_kd);
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
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    for (int i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
        fsm_factory->state_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
        fsm_factory->state_[i].velocity = static_cast<float>(state_interfaces_[i * 3 + 1].get_value());
        fsm_factory->state_[i].torque   = static_cast<float>(state_interfaces_[i * 3 + 2].get_value());
    }

    bool ret = fsm_factory->run();
    if (!ret) {
        for (int i = 0; i < joints_name.size(); i++) { // 安全保护
            command_interfaces_[i * 6 + 0].set_value(state_interfaces_[i * 3 + 0].get_value());
            command_interfaces_[i * 6 + 1].set_value(0.0);
            command_interfaces_[i * 6 + 2].set_value(0.0);
            command_interfaces_[i * 6 + 3].set_value(static_cast<float>(state_interfaces_[i * 6 + 3].get_value()));
            command_interfaces_[i * 6 + 4].set_value(static_cast<float>(state_interfaces_[i * 6 + 4].get_value()));
            command_interfaces_[i * 6 + 5].set_value(0.0f);
        }
        return controller_interface::return_type::ERROR;
    }

    for (int i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
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
