#include "kinamic.hpp"

#include <cmath>
#include <stdexcept>

namespace {

constexpr Eigen::Index kCartesianDimension = 6;
constexpr int kMaxIterations                = 200;
constexpr double kTolerance                 = 1e-6;
constexpr double kDamping                   = 1e-4;
constexpr double kMaxJointStep              = 0.1;
constexpr double kPi                         = 3.14159265358979323846;

Eigen::VectorXd pose_to_cartesian(const Eigen::Isometry3d& pose) {
    const Eigen::Vector3d yaw_pitch_roll = pose.rotation().eulerAngles(2, 1, 0);

    Eigen::VectorXd cartesian(kCartesianDimension);
    cartesian << pose.translation().x(), pose.translation().y(), pose.translation().z(),
        yaw_pitch_roll[2], yaw_pitch_roll[1], yaw_pitch_roll[0];
    return cartesian;
}

double angle_error(double target, double current) {
    return std::remainder(target - current, 2.0 * kPi);
}

}  // namespace

IKSolver::IKSolver(ModelBase* robot, TaskMapping* task_mapping)
    : robot_(robot), task_mapping_(task_mapping) {
    if (robot_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a model");
    }
    if (task_mapping_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a TaskMapping");
    }
}

Eigen::MatrixXd IKSolver::jacobian(const Eigen::VectorXd& joint_pos) const {
    const Eigen::MatrixXd geometric_jacobian = robot_->geometric_jacobian(joint_pos);
    if (geometric_jacobian.rows() != 6 || geometric_jacobian.cols() != robot_->dof()
        || !geometric_jacobian.allFinite()) {
        return {};
    }

    Eigen::MatrixXd task_jacobian;
    if (!task_mapping_->jacobian_map(joint_pos, geometric_jacobian, &task_jacobian)
        || task_jacobian.cols() != geometric_jacobian.cols() || !task_jacobian.allFinite()) {
        return {};
    }

    return task_jacobian;
}

Eigen::VectorXd IKSolver::cartesian_position(const Eigen::VectorXd& joint_pos) const {
    return pose_to_cartesian(robot_->forward_kinematics(joint_pos));
}

bool IKSolver::solve(const Eigen::VectorXd &cart_pos,Eigen::VectorXd &joint_pos) {
    if (problem.size() != kCartesianDimension || !problem.allFinite()) {
        return false;
    }

    const Eigen::VectorXd target = problem;
    const int joint_count = robot_->dof();
    if (joint_count <= 0) {
        return false;
    }

    const Eigen::VectorXd lower = robot_->lower_jointLimit();
    const Eigen::VectorXd upper = robot_->upper_jointLimit();
    const bool use_joint_limits = lower.size() == joint_count && upper.size() == joint_count
        && lower.allFinite() && upper.allFinite();

    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(joint_count);
    if (use_joint_limits) {
        joint_pos = joint_pos.cwiseMax(lower).cwiseMin(upper);
    }

    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        const Eigen::VectorXd current = cartesian_position(joint_pos);
        Eigen::VectorXd error        = target - current;
        for (Eigen::Index i = 3; i < kCartesianDimension; ++i) {
            error[i] = angle_error(target[i], current[i]);
        }

        if (!error.allFinite()) {
            return false;
        }
        if (error.norm() < kTolerance) {
            problem = joint_pos;
            return true;
        }

        const Eigen::MatrixXd task_jacobian = jacobian(joint_pos);
        if (task_jacobian.rows() != problem.size() || task_jacobian.cols() != joint_count) {
            return false;
        }

        const Eigen::MatrixXd hessian = task_jacobian.transpose() * task_jacobian
            + kDamping * kDamping * Eigen::MatrixXd::Identity(joint_count, joint_count);
        const Eigen::VectorXd gradient = task_jacobian.transpose() * error;
        Eigen::VectorXd joint_step      = hessian.ldlt().solve(gradient);
        if (!joint_step.allFinite()) {
            return false;
        }

        joint_step = joint_step.cwiseMax(-kMaxJointStep).cwiseMin(kMaxJointStep);
        if (use_joint_limits) {
            joint_step = joint_step.cwiseMax(lower - joint_pos).cwiseMin(upper - joint_pos);
        }
        joint_pos += joint_step;
    }

    problem = joint_pos;
    const Eigen::VectorXd current = cartesian_position(joint_pos);
    Eigen::VectorXd residual      = target - current;
    for (Eigen::Index i = 3; i < kCartesianDimension; ++i) {
        residual[i] = angle_error(target[i], current[i]);
    }
    return residual.norm() < kTolerance;
}
