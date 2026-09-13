#pragma once

#include "modelbase.hpp"
#include "task.hpp"

#include <Eigen/Dense>
#include <vector>

class IKSolver {
public:
    IKSolver(ModelBase* robot, TaskMapping* task_mapping);
    bool solve(Eigen::VectorXd& problem);
private:
    ModelBase* robot_;
    TaskMapping* task_mapping_;
};
