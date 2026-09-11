#pragma once

#include "modelbase.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>


class TaskUnit {
public:
    // AxisDot constrains the dot product between a tool-frame axis and a
    // world-frame reference axis. Its target is a scalar in [-1, 1].
    enum TaskType { PositionX = 0, PositionY = 1, PositionZ = 2, AxisDot = 3 };

    TaskType type;
    double target{0.0};
    double weight{1.0};

    // These fields are used only when type == AxisDot. They do not need to
    // be unit length; the solver normalizes them before evaluating the task.
    Eigen::Vector3d tool_axis{Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d reference_axis{Eigen::Vector3d::UnitZ()};
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
    Eigen::Vector3d normalized_or_zero(const Eigen::Vector3d& axis) const;

    ModelBase* robot_;
    Eigen::VectorXd yq;
};
