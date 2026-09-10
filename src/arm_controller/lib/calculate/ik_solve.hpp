#pragma once

#include "ik_task.hpp"


class TaskEvaluator {
public:
    explicit TaskEvaluator(const ModelBase& robot);
    Eigen::VectorXd value(const Eigen::VectorXd& q, const Task& task) const;
    Eigen::VectorXd error(const Eigen::VectorXd& q, const Task& task) const;
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q, const Task& task) const;

private:
    const ModelBase& robot_;
};


class IKProblem {
public:
    Eigen::VectorXd initial_q;
    Task task;

    bool enable_joint_limits = true;
    bool enable_step_limits  = false;
    Eigen::VectorXd max_step;
};

class IKSolver {
public:
    explicit IKSolver(const ModelBase& robot);
    ~IKSolver()  = default;
    IKResult solve(const IKProblem& problem);
private:
    const ModelBase& robot_;
};