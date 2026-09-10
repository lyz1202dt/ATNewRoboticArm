#pragma once

#include "modelbase.hpp"
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
    int dimension() const;
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
