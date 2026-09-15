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

bool IKSolver::task_position(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_position) {
    if (task_position == nullptr) {
        return false;
    }

    if (!task_mapping_->position_map(joint_pos, robot_->forward_kinematics(joint_pos), task_position)
        || task_position->size() == 0 || !task_position->allFinite()) {
        task_position->resize(0);
        return false;
    }
    return true;
}

bool IKSolver::jacobian(const Eigen::VectorXd& joint_pos, Eigen::MatrixXd* task_jacobian) {
    if (task_jacobian == nullptr) {
        return false;
    }

    if (!robot_->geometric_jacobian(joint_pos, &workspace_.geometric_jacobian)
        || workspace_.geometric_jacobian.rows() != 6
        || workspace_.geometric_jacobian.cols() != robot_->dof()
        || !workspace_.geometric_jacobian.allFinite()) {
        task_jacobian->resize(0, 0);
        return false;
    }

    if (!task_mapping_->jacobian_map(joint_pos, workspace_.geometric_jacobian, task_jacobian)
        || task_jacobian->rows() == 0 || task_jacobian->cols() != workspace_.geometric_jacobian.cols()
        || !task_jacobian->allFinite()) {
        task_jacobian->resize(0, 0);
        return false;
    }

    return true;
}

bool IKSolver::solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos) {

    const int joint_count = robot_->dof();
    workspace_.solution = joint_pos;
    const Eigen::VectorXd& lower = robot_->lower_jointLimit();
    const Eigen::VectorXd& upper = robot_->upper_jointLimit();
    const bool use_joint_limits = lower.size() == joint_count && upper.size() == joint_count
        && lower.allFinite() && upper.allFinite();

    if (!workspace_.solution.allFinite()) {
        return false;
    }
    if (use_joint_limits) {
        workspace_.solution = workspace_.solution.cwiseMax(lower).cwiseMin(upper);
    }

    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        if (!task_position(workspace_.solution, &workspace_.current) || workspace_.current.size() != target.size()) {
            return false;
        }

        workspace_.error.resize(target.size());
        workspace_.error.noalias() = target - workspace_.current;
        if (!workspace_.error.allFinite()) {
            return false;
        }
        if (workspace_.error.norm() < kTolerance) {
            joint_pos = workspace_.solution;
            return true;
        }

        if (!jacobian(workspace_.solution, &workspace_.task_jacobian)
            || workspace_.task_jacobian.rows() != target.size()
            || workspace_.task_jacobian.cols() != joint_count) {
            return false;
        }

        workspace_.hessian.resize(joint_count, joint_count);
        workspace_.hessian.noalias() = workspace_.task_jacobian.transpose() * workspace_.task_jacobian;
        workspace_.hessian.diagonal().array() += kDamping * kDamping;

        workspace_.rhs.resize(joint_count);
        workspace_.rhs.noalias() = workspace_.task_jacobian.transpose() * workspace_.error;
        workspace_.ldlt.compute(workspace_.hessian);
        workspace_.joint_step = workspace_.ldlt.solve(workspace_.rhs);
        if (!workspace_.joint_step.allFinite()) {
            return false;
        }

        workspace_.joint_step = workspace_.joint_step.cwiseMax(-kMaxJointStep).cwiseMin(kMaxJointStep);
        if (use_joint_limits) {
            workspace_.joint_step =
                workspace_.joint_step.cwiseMax(lower - workspace_.solution).cwiseMin(upper - workspace_.solution);
        }
        workspace_.solution += workspace_.joint_step;
    }

    if (task_position(workspace_.solution, &workspace_.current)
        && workspace_.current.size() == target.size()
        && (target - workspace_.current).norm() < kTolerance) {
        joint_pos = workspace_.solution;
        return true;
    }
    return false;
}
