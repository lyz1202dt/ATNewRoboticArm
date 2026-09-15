#include "kinamic.hpp"

#include <stdexcept>
#include <utility>

namespace {

constexpr int kMaxIterations     = 200;
constexpr double kTolerance      = 1e-6;
constexpr double kDamping        = 1e-4;
constexpr double kMaxJointStep   = 0.1;

}  // namespace

IKSolver::IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping)
    : robot_(std::move(robot)), task_mapping_(std::move(task_mapping)) {
    if (robot_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a model");
    }
    if (task_mapping_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a TaskMapping");
    }
}

Eigen::VectorXd IKSolver::task_position(const Eigen::VectorXd& joint_pos) {

    Eigen::VectorXd position;
    if (!task_mapping_->position_map(joint_pos, robot_->forward_kinematics(joint_pos), &position)
        || position.size() == 0 || !position.allFinite()) {
        return {};
    }
    return position;
}

Eigen::MatrixXd IKSolver::jacobian(const Eigen::VectorXd& joint_pos) {

    const Eigen::MatrixXd geometric_jacobian = robot_->geometric_jacobian(joint_pos);
    if (geometric_jacobian.rows() != 6 || geometric_jacobian.cols() != robot_->dof()
        || !geometric_jacobian.allFinite()) {
        return {};
    }

    Eigen::MatrixXd task_jacobian;
    if (!task_mapping_->jacobian_map(joint_pos, geometric_jacobian, &task_jacobian)
        || task_jacobian.rows() == 0 || task_jacobian.cols() != geometric_jacobian.cols()
        || !task_jacobian.allFinite()) {
        return {};
    }

    return task_jacobian;
}

bool IKSolver::solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos) {

    const int joint_count = robot_->dof();
    Eigen::VectorXd solution = joint_pos;
    const Eigen::VectorXd lower = robot_->lower_jointLimit();
    const Eigen::VectorXd upper = robot_->upper_jointLimit();
    const bool use_joint_limits = lower.size() == joint_count && upper.size() == joint_count
        && lower.allFinite() && upper.allFinite();

    if (!solution.allFinite()) {
        return false;
    }
    if (use_joint_limits) {
        solution = solution.cwiseMax(lower).cwiseMin(upper);
    }

    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        const Eigen::VectorXd current = task_position(solution);
        if (current.size() != target.size()) {
            return false;
        }

        const Eigen::VectorXd error = target - current;
        if (!error.allFinite()) {
            return false;
        }
        if (error.norm() < kTolerance) {
            joint_pos = solution;
            return true;
        }

        const Eigen::MatrixXd task_jacobian = jacobian(solution);
        if (task_jacobian.rows() != target.size() || task_jacobian.cols() != joint_count) {
            return false;
        }

        const Eigen::MatrixXd hessian = task_jacobian.transpose() * task_jacobian
            + kDamping * kDamping * Eigen::MatrixXd::Identity(joint_count, joint_count);
        Eigen::VectorXd joint_step = hessian.ldlt().solve(task_jacobian.transpose() * error);
        if (!joint_step.allFinite()) {
            return false;
        }

        joint_step = joint_step.cwiseMax(-kMaxJointStep).cwiseMin(kMaxJointStep);
        if (use_joint_limits) {
            joint_step = joint_step.cwiseMax(lower - solution).cwiseMin(upper - solution);
        }
        solution += joint_step;
    }

    const Eigen::VectorXd current = task_position(solution);
    if (current.size() == target.size() && (target - current).norm() < kTolerance) {
        joint_pos = solution;
        return true;
    }
    return false;
}
