#include "ik_solve.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <pinocchio/algorithm/frames.hxx>

void Task::add(const TaskUnit& task_unit) {
    task_units_.push_back(task_unit);
}

void Task::set(const std::vector<TaskUnit>& task_units) {
    task_units_ = task_units;
}

void Task::clear() {
    task_units_.clear();
}


const std::vector<TaskUnit>& Task::components() const {
    return task_units_;
}

IKSolver::IKSolver(ModelBase* robot) {
    robot_ = robot;
}

Eigen::MatrixXd IKSolver::jacobian(const Eigen::VectorXd& q, const Task& task) {
    const Eigen::MatrixXd J = robot_->geometric_jacobian(q);

    const Eigen::Matrix3d R = robot_->forward_kinematics(q).rotation();

    // Eigen eulerAngles(2, 1, 0):
    // [0] = yaw, [1] = pitch, [2] = roll
    const Eigen::Vector3d euler_zyx = R.eulerAngles(2, 1, 0);

    const double yaw   = euler_zyx[0];
    const double pitch = euler_zyx[1];

    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);

    // clang-format off
    Eigen::Matrix3d E_world;
    E_world <<
        cy * cp, -sy, 0.0,
        sy * cp,  cy, 0.0,
        -sp,      0.0, 1.0;
    // clang-format on

    const Eigen::MatrixXd J_rpy = E_world.colPivHouseholderQr().solve(J.bottomRows(3));

    Eigen::MatrixXd J_task(static_cast<Eigen::Index>(task.components().size()), J.cols());

    for (Eigen::Index i = 0; i < J_task.rows(); ++i) {
        switch (task.components()[i].type) {
        case TaskUnit::PositionX: J_task.row(i) = J.row(0); break;

        case TaskUnit::PositionY: J_task.row(i) = J.row(1); break;

        case TaskUnit::PositionZ: J_task.row(i) = J.row(2); break;

        case TaskUnit::Roll: J_task.row(i) = J_rpy.row(0); break;

        case TaskUnit::Pitch: J_task.row(i) = J_rpy.row(1); break;

        case TaskUnit::YAW: J_task.row(i) = J_rpy.row(2); break;

        default: throw std::invalid_argument("Unknown task type.");
        }
    }

    return J_task;
}

IKResult IKSolver::solve(const IKProblem& problem) {
    Eigen::VectorXd target;
    target.resize(problem.task.components().size());
    for (int i = 0; i < target.size(); i++)
        target[i] = problem.task.components()[i].target;
    auto cart_pos = robot_->forward_kinematics(problem.initial_q);
}
