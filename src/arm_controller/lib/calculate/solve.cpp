#include "solve.hpp"

#include <sstream>
#include <stdexcept>

namespace {

constexpr Eigen::Index kCartesianDimension = 6;
constexpr double kJacobianDifferenceStep    = 1e-6;

Eigen::MatrixXd task_jacobian(const IKSolver& ik_solver, const Eigen::VectorXd& joint_pos) {
    const Eigen::MatrixXd jacobian = ik_solver.jacobian(joint_pos);
    if (jacobian.cols() != joint_pos.size() || jacobian.rows() == 0 || !jacobian.allFinite()) {
        throw std::runtime_error("TaskMapping returned an invalid Jacobian");
    }
    return jacobian;
}

}  // namespace

ArmSolve::ArmSolve(ModelBase* model, TaskMapping* task_mapping)
    : model_(model), ik_solver_(model, task_mapping) {
}

Eigen::VectorXd ArmSolve::inverse_kinamic(const Eigen::VectorXd& cart_pos) {
    Eigen::VectorXd joint_pos = cart_pos;
    if (!ik_solver_.solve(joint_pos)) {
        return {};
    }
    return joint_pos;
}

Eigen::VectorXd ArmSolve::forward_kinamic(const Eigen::VectorXd& joint_pos) {
    return ik_solver_.cartesian_position(joint_pos);
}

Eigen::VectorXd ArmSolve::inverse_dynamic(const Eigen::VectorXd& joint_pos,
                                          const Eigen::VectorXd& cart_vel,
                                          const Eigen::VectorXd& cart_acc,
                                          const Eigen::VectorXd& cart_force) {
    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint position size: expected " << joint_count << ", got " << joint_pos.size();
        throw std::invalid_argument(message.str());
    }

    Eigen::VectorXd joint_vel;
    Eigen::VectorXd joint_acc;
    Eigen::MatrixXd jacobian;

    if (cart_vel.size() == 0 && cart_acc.size() == 0) {
        joint_vel = Eigen::VectorXd::Zero(joint_count);
        joint_acc = Eigen::VectorXd::Zero(joint_count);
    } else if (cart_vel.size() == kCartesianDimension && cart_acc.size() == kCartesianDimension) {
        jacobian = task_jacobian(ik_solver_, joint_pos);
        if (jacobian.rows() != kCartesianDimension) {
            throw std::invalid_argument("TaskMapping must return a 6-row Jacobian for Cartesian dynamics");
        }

        const auto jacobian_solver = jacobian.completeOrthogonalDecomposition();
        joint_vel                            = jacobian_solver.solve(cart_vel);
        const Eigen::MatrixXd next_jacobian = task_jacobian(
            ik_solver_, joint_pos + kJacobianDifferenceStep * joint_vel);
        const Eigen::VectorXd jacobian_velocity =
            ((next_jacobian - jacobian) / kJacobianDifferenceStep) * joint_vel;
        joint_acc = jacobian_solver.solve(cart_acc - jacobian_velocity);
    } else if (cart_vel.size() == joint_count && cart_acc.size() == joint_count) {
        joint_vel = cart_vel;
        joint_acc = cart_acc;
    } else {
        std::ostringstream message;
        message << "Invalid velocity/acceleration sizes: expected either " << joint_count
                << " joint values or 6 Cartesian values, got " << cart_vel.size() << " and "
                << cart_acc.size();
        throw std::invalid_argument(message.str());
    }

    if (!joint_vel.allFinite() || !joint_acc.allFinite()) {
        throw std::runtime_error("Unable to map velocity or acceleration to joint space");
    }

    Eigen::VectorXd joint_torque = model_->inverse_dynamic(joint_pos, joint_vel, joint_acc);
    if (joint_torque.size() != joint_count || !joint_torque.allFinite()) {
        throw std::runtime_error("Model returned an invalid torque vector");
    }

    if (cart_force.size() == 0) {
        return joint_torque;
    }

    if (jacobian.rows() == 0) {
        jacobian = task_jacobian(ik_solver_, joint_pos);
    }
    if (cart_force.size() != jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid Cartesian force size: expected " << jacobian.rows() << ", got "
                << cart_force.size();
        throw std::invalid_argument(message.str());
    }

    joint_torque += jacobian.transpose() * cart_force;
    return joint_torque;
}

Eigen::VectorXd ArmSolve::static_force(const Eigen::VectorXd& joint_pos,
                                       const Eigen::VectorXd& joint_torque_residual) {
    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        throw std::invalid_argument("Invalid joint position size");
    }
    if (joint_torque_residual.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint torque residual size: expected " << joint_count << ", got "
                << joint_torque_residual.size();
        throw std::invalid_argument(message.str());
    }

    const Eigen::MatrixXd jacobian = task_jacobian(ik_solver_, joint_pos);
    return jacobian.transpose().completeOrthogonalDecomposition().solve(joint_torque_residual);
}
