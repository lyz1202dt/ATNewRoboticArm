#pragma once

#include "kinamic.hpp"
#include <memory>
#include <Eigen/Dense>

class ArmSolve {
public:
    ArmSolve(std::shared_ptr<ModelBase> model, std::shared_ptr<TaskMapping> task_mapping);

    // 任务空间目标到关节空间解。
    bool inverse_kinamic(const Eigen::VectorXd& task_pos, Eigen::VectorXd* joint_pos);

    // 关节空间状态到用户定义的任务空间状态。
    bool forward_kinamic(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_pos);

    // 使用用户定义任务空间速度、加速度和力计算关节力矩。
    bool inverse_dynamic(const Eigen::VectorXd& joint_pos,
                         const Eigen::VectorXd& task_vel,
                         const Eigen::VectorXd& task_acc,
                         const Eigen::VectorXd& task_force,
                         Eigen::VectorXd* joint_torque);

    // 根据关节力矩残差计算用户定义任务空间受力。
    bool static_force(const Eigen::VectorXd& joint_pos,
                      const Eigen::VectorXd& joint_torque_residual,
                      Eigen::VectorXd* task_force);

private:
    struct Workspace {
        Eigen::VectorXd finite_joint_pos;
        Eigen::VectorXd joint_vel;
        Eigen::VectorXd jacobian_velocity;
        Eigen::VectorXd task_acc_rhs;
        Eigen::VectorXd joint_acc;
        Eigen::VectorXd task_force_torque;
        Eigen::MatrixXd jacobian;
        Eigen::MatrixXd next_jacobian;
        Eigen::MatrixXd jacobian_delta;
        Eigen::MatrixXd jacobian_transpose;
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> jacobian_solver;
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> static_force_solver;
    };

    std::shared_ptr<ModelBase> model_;
    std::shared_ptr<IKSolver> ik_solver_;
    Workspace workspace_;
};
