#include "ik_task.hpp"

void Task::add(const TaskUnit& task_unit) {
    task_units_.push_back(task_unit);
}

void Task::set(const std::vector<TaskUnit>& task_units) {
    task_units_ = task_units;
}

void Task::clear() {
    task_units_.clear();
}

int Task::dimension() const {
    return static_cast<int>(task_units_.size());
}

const std::vector<TaskUnit>& Task::components() const {
    return task_units_;
}

