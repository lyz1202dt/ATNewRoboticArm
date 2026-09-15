#pragma once

#include "modelbase.hpp"
#include "task.hpp"
#include <memory>
#include <Eigen/Dense>

class IKSolver {
public:
    IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping);

    bool solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos);

    bool jacobian(const Eigen::VectorXd& joint_pos, Eigen::MatrixXd* task_jacobian);

    bool task_position(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_position);

private:
    struct Workspace {
        Eigen::VectorXd solution;
        Eigen::VectorXd current;
        Eigen::VectorXd error;
        Eigen::VectorXd rhs;
        Eigen::VectorXd joint_step;
        Eigen::MatrixXd geometric_jacobian;
        Eigen::MatrixXd task_jacobian;
        Eigen::MatrixXd hessian;
        Eigen::LDLT<Eigen::MatrixXd> ldlt;
    };

    std::shared_ptr<ModelBase> robot_;
    std::shared_ptr<TaskMapping> task_mapping_;
    Workspace workspace_;
};
