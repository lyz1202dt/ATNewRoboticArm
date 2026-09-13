#pragma once

#include "kinamic.hpp"

#include <Eigen/Dense>

class ArmSolve {
public:
    ArmSolve(ModelBase* model, TaskMapping* task_mapping);

    //运动学逆解
    Eigen::VectorXd inverse_kinamic(const Eigen::VectorXd& cart_pos);

    //运动学正解
    Eigen::VectorXd forward_kinamic(const Eigen::VectorXd& joint_pos);

    //逆动力学，计算关节力矩
    Eigen::VectorXd inverse_dynamic(const Eigen::VectorXd& joint_pos,
                                     const Eigen::VectorXd& cart_vel,
                                     const Eigen::VectorXd& cart_acc,
                                     const Eigen::VectorXd& cart_force);

    //力矩映射，根据力矩残差计算任务空间受力。
    Eigen::VectorXd static_force(const Eigen::VectorXd& joint_pos,
                                 const Eigen::VectorXd& joint_torque_residual);

private:
    ModelBase* model_;
    IKSolver ik_solver_;
};
