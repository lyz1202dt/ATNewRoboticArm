#pragma once

#include "kinamic.hpp"
#include <memory>
#include <Eigen/Dense>

class ArmSolve {
public:
    ArmSolve(std::shared_ptr<ModelBase> model, std::shared_ptr<TaskMapping> task_mapping);

    // 任务空间目标到关节空间解。
    Eigen::VectorXd inverse_kinamic(const Eigen::VectorXd& task_pos);

    // 关节空间状态到用户定义的任务空间状态。
    Eigen::VectorXd forward_kinamic(const Eigen::VectorXd& joint_pos);

    // 使用用户定义任务空间速度、加速度和力计算关节力矩。
    Eigen::VectorXd inverse_dynamic(const Eigen::VectorXd& joint_pos,
                                     const Eigen::VectorXd& task_vel,
                                     const Eigen::VectorXd& task_acc,
                                     const Eigen::VectorXd& task_force);

    // 根据关节力矩残差计算用户定义任务空间受力。
    Eigen::VectorXd static_force(const Eigen::VectorXd& joint_pos,
                                 const Eigen::VectorXd& joint_torque_residual);

private:
    std::shared_ptr<ModelBase> model_;
    std::shared_ptr<IKSolver> ik_solver_;
};
