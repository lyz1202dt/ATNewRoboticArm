#pragma once

#include "modelbase.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <vector>


class TaskUnit {
public:
    enum TaskType { PositionX, PositionY, PositionZ, Roll, Pitch, YAW };

    TaskType type;
    double target{0.0};
    double weight{1.0};
    bool hard_constraint{false};
};

class Task {
public:
    void add(const TaskUnit& task_unit);
    void set(const std::vector<TaskUnit>& task_units);
    void clear();
    const std::vector<TaskUnit>& components() const;

private:
    std::vector<TaskUnit> task_units_;
};

struct IKResult {
    bool success = false;
    Eigen::VectorXd q;
    double error_norm = 0.0;
    int iterations    = 0;
};

struct IKProblem {
    Eigen::VectorXd initial_q;
    Task task;

    bool enable_joint_limits = true;
    bool enable_step_limits  = false;
    Eigen::VectorXd max_step;
};

class IKSolver {
public:
    explicit IKSolver(ModelBase* robot);
    ~IKSolver() = default;
    IKResult solve(const IKProblem& problem);

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q, const Task& task);

private:
    ModelBase* robot_;
    Eigen::VectorXd yq;
};