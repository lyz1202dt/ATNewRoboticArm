#pragma once

#include "modelbase.hpp"
#include "task.hpp"
#include <memory>
#include <Eigen/Dense>

class IKSolver {
public:
    IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping);

    bool solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos);

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& joint_pos);

    Eigen::VectorXd task_position(const Eigen::VectorXd& joint_pos);

private:
    ModelBase* robot_;
    TaskMapping* task_mapping_;
};
