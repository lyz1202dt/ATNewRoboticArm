#pragma once

#include <Eigen/Dense>
#include "ik_solve.hpp"

class ArmSolve{
public:
    explicit ArmSolve(ModelBase* model);

    //设置当前求解器使用的工作空间
    void set_cart_task_type(const std::vector<TaskUnit> &task);

    //运动学逆解(来自IKSolver)
    Eigen::VectorXd inverse_kinamic(Eigen::VectorXd cart_pos);

    //运动学正解(直接来自ModelBase方法)
    Eigen::VectorXd forward_kinamic(Eigen::VectorXd joint_pos);

    //逆动力学，计算关节力矩(直接来自ModelBase方法)
    Eigen::VectorXd inverse_dynamic(Eigen::VectorXd joint_pos,Eigen::VectorXd cart_vel,Eigen::VectorXd cart_acc,Eigen::VectorXd cart_force);

    //力矩映射，根据力矩残差计算机械臂末端受力(使用来自IKSolver的雅可比矩阵计算)
    Eigen::VectorXd static_force(Eigen::VectorXd joint_pos,Eigen::VectorXd joint_torque_residual);
private:
    ModelBase* model_;
    IKSolver ik_solver_;
    std::vector<TaskUnit> task_;
    Eigen::VectorXd last_joint_pos_;
};
