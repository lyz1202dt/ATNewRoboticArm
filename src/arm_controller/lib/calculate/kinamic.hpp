#pragma once

#include "modelbase.hpp"
#include "task.hpp"

#include <Eigen/Dense>

class IKSolver {
public:
    IKSolver(ModelBase* robot, TaskMapping* task_mapping);

    bool solve(const Eigen::VectorXd &cart_pos,Eigen::VectorXd &joint_pos);

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& joint_pos) const;

    Eigen::VectorXd cartesian_position(const Eigen::VectorXd& joint_pos) const;

private:
    ModelBase* robot_;
    TaskMapping* task_mapping_;
};
