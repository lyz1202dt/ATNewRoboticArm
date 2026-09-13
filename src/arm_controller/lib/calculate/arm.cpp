#include "arm.hpp"

#include <sstream>
#include <stdexcept>

ArmSolve::ArmSolve(ModelBase* model, TaskMapping* task_mapping)
    : model_(model) {
    ik_solver_ = new IKSolver(model, task_mapping);
}

Eigen::VectorXd ArmSolve::inverse_kinamic(const Eigen::VectorXd& task_pos) {
    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(model_->dof());
    if (!ik_solver_->solve(task_pos, joint_pos)) {
        return {};
    }
    return joint_pos;
}

Eigen::VectorXd ArmSolve::forward_kinamic(const Eigen::VectorXd& joint_pos) {
    return ik_solver_->task_position(joint_pos);
}

Eigen::VectorXd ArmSolve::inverse_dynamic(
    const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& task_vel, const Eigen::VectorXd& task_acc, const Eigen::VectorXd& task_force) {
    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint position size: expected " << joint_count << ", got " << joint_pos.size();
        throw std::invalid_argument(message.str());
    }

    constexpr double kJacobianDifferenceStep = 1e-6;

    Eigen::VectorXd joint_vel;
    Eigen::VectorXd joint_acc;
    Eigen::MatrixXd jacobian;

    const auto jacobian_solver              = jacobian.completeOrthogonalDecomposition();
    joint_vel                               = jacobian_solver.solve(task_vel);
    const Eigen::MatrixXd next_jacobian     = ik_solver_->jacobian(joint_pos + kJacobianDifferenceStep * joint_vel);
    const Eigen::VectorXd jacobian_velocity = ((next_jacobian - jacobian) / kJacobianDifferenceStep) * joint_vel;
    joint_acc                               = jacobian_solver.solve(task_acc - jacobian_velocity);

    if (!joint_vel.allFinite() || !joint_acc.allFinite()) {
        throw std::runtime_error("Unable to map velocity or acceleration to joint space");
    }

    Eigen::VectorXd joint_torque = model_->inverse_dynamic(joint_pos, joint_vel, joint_acc);
    joint_torque += jacobian.transpose() * task_force;
    return joint_torque;
}

Eigen::VectorXd ArmSolve::static_force(const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_torque_residual) {
    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        throw std::invalid_argument("Invalid joint position size");
    }
    if (joint_torque_residual.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint torque residual size: expected " << joint_count << ", got " << joint_torque_residual.size();
        throw std::invalid_argument(message.str());
    }

    return ik_solver_->jacobian(joint_pos).transpose().completeOrthogonalDecomposition().solve(joint_torque_residual);
}
