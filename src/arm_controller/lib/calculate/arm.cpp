#include "arm.hpp"

#include <sstream>
#include <stdexcept>

ArmSolve::ArmSolve(std::shared_ptr<ModelBase> model, std::shared_ptr<TaskMapping> task_mapping)
    : model_(model) {
    ik_solver_ = std::make_shared<IKSolver>(model, task_mapping);
}

bool ArmSolve::inverse_kinamic(const Eigen::VectorXd& task_pos, Eigen::VectorXd* joint_pos) {
    if (joint_pos == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos->size() != joint_count) {
        joint_pos->resize(joint_count);
    }
    joint_pos->setZero();
    return ik_solver_->solve(task_pos, *joint_pos);
}

bool ArmSolve::forward_kinamic(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_pos) {
    return ik_solver_->task_position(joint_pos, task_pos);
}

bool ArmSolve::inverse_dynamic(
    const Eigen::VectorXd& joint_pos,
    const Eigen::VectorXd& task_vel,
    const Eigen::VectorXd& task_acc,
    const Eigen::VectorXd& task_force,
    Eigen::VectorXd* joint_torque) {
    if (joint_torque == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint position size: expected " << joint_count << ", got " << joint_pos.size();
        throw std::invalid_argument(message.str());
    }
    if (!joint_pos.allFinite()) {
        throw std::invalid_argument("Joint position contains non-finite values");
    }

    constexpr double kJacobianDifferenceStep = 1e-6;

    if (!ik_solver_->jacobian(joint_pos, &workspace_.jacobian)) {
        return false;
    }
    if (task_vel.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task velocity size: expected " << workspace_.jacobian.rows() << ", got " << task_vel.size();
        throw std::invalid_argument(message.str());
    }
    if (task_acc.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task acceleration size: expected " << workspace_.jacobian.rows() << ", got " << task_acc.size();
        throw std::invalid_argument(message.str());
    }
    if (task_force.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task force size: expected " << workspace_.jacobian.rows() << ", got " << task_force.size();
        throw std::invalid_argument(message.str());
    }
    workspace_.jacobian_solver.compute(workspace_.jacobian);

    workspace_.joint_vel = workspace_.jacobian_solver.solve(task_vel);
    if (!workspace_.joint_vel.allFinite()) {
        return false;
    }

    // x_ddot = J(q) q_ddot + J_dot(q, q_dot) q_dot.
    workspace_.finite_joint_pos.resize(joint_count);
    workspace_.finite_joint_pos.noalias() = joint_pos + kJacobianDifferenceStep * workspace_.joint_vel;
    if (!ik_solver_->jacobian(workspace_.finite_joint_pos, &workspace_.next_jacobian)) {
        return false;
    }

    workspace_.jacobian_delta.resize(workspace_.jacobian.rows(), workspace_.jacobian.cols());
    workspace_.jacobian_delta.noalias() = workspace_.next_jacobian - workspace_.jacobian;
    workspace_.jacobian_delta /= kJacobianDifferenceStep;

    workspace_.jacobian_velocity.resize(workspace_.jacobian.rows());
    workspace_.jacobian_velocity.noalias() = workspace_.jacobian_delta * workspace_.joint_vel;

    workspace_.task_acc_rhs.resize(task_acc.size());
    workspace_.task_acc_rhs.noalias() = task_acc - workspace_.jacobian_velocity;
    workspace_.joint_acc = workspace_.jacobian_solver.solve(workspace_.task_acc_rhs);
    if (!workspace_.joint_acc.allFinite()) {
        return false;
    }

    if (!model_->inverse_dynamic(joint_pos, workspace_.joint_vel, workspace_.joint_acc, joint_torque)) {
        return false;
    }

    workspace_.task_force_torque.resize(joint_count);
    workspace_.task_force_torque.noalias() = workspace_.jacobian.transpose() * task_force;
    *joint_torque += workspace_.task_force_torque;
    return joint_torque->allFinite();
}

bool ArmSolve::static_force(const Eigen::VectorXd& joint_pos,
                            const Eigen::VectorXd& joint_torque_residual,
                            Eigen::VectorXd* task_force) {
    if (task_force == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        throw std::invalid_argument("Invalid joint position size");
    }
    if (joint_torque_residual.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint torque residual size: expected " << joint_count << ", got " << joint_torque_residual.size();
        throw std::invalid_argument(message.str());
    }

    if (!ik_solver_->jacobian(joint_pos, &workspace_.jacobian)) {
        return false;
    }

    workspace_.jacobian_transpose.resize(workspace_.jacobian.cols(), workspace_.jacobian.rows());
    workspace_.jacobian_transpose.noalias() = workspace_.jacobian.transpose();
    workspace_.static_force_solver.compute(workspace_.jacobian_transpose);
    *task_force = workspace_.static_force_solver.solve(joint_torque_residual);
    return task_force->allFinite();
}
