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

struct IKResult {
    bool success = false;
    Eigen::VectorXd q;
    double error_norm = 0.0;
    int iterations    = 0;
};

struct IKProblem {
    Eigen::VectorXd initial_q;
    std::vector<TaskUnit> task;

    bool enable_joint_limits = true;
    bool enable_step_limits  = false;
    Eigen::VectorXd max_step;
};

class IKSolver {
public:
    explicit IKSolver(ModelBase* robot);
    IKResult solve(const IKProblem& problem);
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q, const std::vector<TaskUnit> &task);
private:
    Eigen::Vector3d normalized_or_zero(const Eigen::Vector3d& axis) const;

    ModelBase* robot_;
    Eigen::VectorXd yq;
};

class FKSolver {
public:
    explicit FKSolver(ModelBase* robot);
    IKResult forward_kinamic(const IKProblem& problem);
    Eigen::Vector3d normalized_or_zero(const Eigen::Vector3d& axis) const;
private:
    std::vector<TaskUnit::TaskType> task_type;
    ModelBase* robot_;
};