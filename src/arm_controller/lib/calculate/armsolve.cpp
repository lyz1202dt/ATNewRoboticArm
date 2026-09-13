#include "armsolve.hpp"

#include <limits>
#include <sstream>
#include <stdexcept>

ArmSolve::ArmSolve(ModelBase* model)
    : model_(model),
    ik_solver_(model){
    last_joint_pos_ = Eigen::VectorXd::Zero(model_->dof());
}

void ArmSolve::set_cart_task_type(const std::vector<TaskUnit> &task) {
    task_ = task;
}

Eigen::VectorXd ArmSolve::inverse_kinamic(Eigen::VectorXd cart_pos) {
    for(int i=0;i<task_.size();i++)     //更新目标
        task_[i].target=cart_pos[i];
    IKProblem ikproblem;
    ikproblem.task=task_;
    ikproblem.initial_q=Eigen::VectorXd::Zero(task_.size());
    ik_solver_.solve(ikproblem);
    Eigen::VectorXd result;
    return result;
}

Eigen::VectorXd ArmSolve::forward_kinamic(Eigen::VectorXd joint_pos) {
    const Eigen::Isometry3d pose = model_->forward_kinematics(joint_pos);

    Eigen::VectorXd cart_pos(static_cast<Eigen::Index>(task_.size()));
    for (Eigen::Index i = 0; i < cart_pos.size(); ++i) {
        const TaskUnit& component = task_[static_cast<std::size_t>(i)];
        switch (component.type) {
        case TaskUnit::PositionX:
        case TaskUnit::PositionY:
        case TaskUnit::PositionZ:
            cart_pos[i] = pose.translation()[static_cast<Eigen::Index>(component.type)];
            break;
        case TaskUnit::AxisDot: {
            constexpr double kAxisEpsilon = 1e-12;
            const double tool_axis_norm   = component.tool_axis.norm();
            const double reference_norm   = component.reference_axis.norm();
            if (tool_axis_norm <= kAxisEpsilon || reference_norm <= kAxisEpsilon) {
                cart_pos[i] = 0.0;
                break;
            }
            const Eigen::Vector3d tool_axis       = component.tool_axis / tool_axis_norm;
            const Eigen::Vector3d reference_axis  = component.reference_axis / reference_norm;
            cart_pos[i]                          = reference_axis.dot(pose.rotation() * tool_axis);
            break;
        }
        default:
            cart_pos[i] = std::numeric_limits<double>::quiet_NaN();
            break;
        }
    }
    return cart_pos;
}

Eigen::VectorXd ArmSolve::inverse_dynamic(Eigen::VectorXd joint_pos,
                                           Eigen::VectorXd cart_vel,
                                           Eigen::VectorXd cart_acc,
                                           Eigen::VectorXd cart_force) {
    if (model_ == nullptr) {
        throw std::runtime_error("ArmSolve has no model");
    }

    const int joint_count = model_->dof();
    const int task_count   = static_cast<int>(task_.size());

    Eigen::VectorXd joint_vel;
    Eigen::VectorXd joint_acc;
    Eigen::MatrixXd task_jacobian;

    if (cart_vel.size() == 0 && cart_acc.size() == 0) {
        joint_vel = Eigen::VectorXd::Zero(joint_count);
        joint_acc = Eigen::VectorXd::Zero(joint_count);
    } else if (task_count > 0 && cart_vel.size() == task_count && cart_acc.size() == task_count) {
        task_jacobian = ik_solver_.jacobian(joint_pos, task_);
        const auto jacobian_solver = task_jacobian.completeOrthogonalDecomposition();
        joint_vel                     = jacobian_solver.solve(cart_vel);

        Eigen::VectorXd task_acceleration = cart_acc;
        if (joint_pos.size() == joint_count) {
            constexpr double kJacobianDifferenceStep = 1e-6;
            const Eigen::MatrixXd next_jacobian =
                ik_solver_.jacobian(joint_pos + kJacobianDifferenceStep * joint_vel, task_);
            const Eigen::VectorXd jacobian_velocity =
                ((next_jacobian - task_jacobian) / kJacobianDifferenceStep) * joint_vel;
            task_acceleration -= jacobian_velocity;
        }
        joint_acc = jacobian_solver.solve(task_acceleration);
    } else if (cart_vel.size() == joint_count && cart_acc.size() == joint_count) {
        joint_vel = cart_vel;
        joint_acc = cart_acc;
    } else {
        std::ostringstream message;
        message << "Invalid velocity/acceleration sizes: expected either " << joint_count
                << " joint values or " << task_count << " task values, got " << cart_vel.size() << " and "
                << cart_acc.size();
        throw std::invalid_argument(message.str());
    }

    if (!joint_vel.allFinite() || !joint_acc.allFinite()) {
        throw std::runtime_error("Unable to map Cartesian velocity or acceleration to joint space");
    }

    Eigen::VectorXd joint_torque = model_->inverse_dynamic(joint_pos, joint_vel, joint_acc);
    if (joint_torque.size() != joint_count) {
        std::ostringstream message;
        message << "Model returned an invalid torque size: expected " << joint_count << ", got "
                << joint_torque.size();
        throw std::runtime_error(message.str());
    }

    if (cart_force.size() == 0) {
        return joint_torque;
    }

    Eigen::MatrixXd force_jacobian;
    if (task_count > 0 && cart_force.size() == task_count) {
        if (task_jacobian.rows() == 0) {
            task_jacobian = ik_solver_.jacobian(joint_pos, task_);
        }
        force_jacobian = task_jacobian;
    } else if (cart_force.size() == 6) {
        force_jacobian = model_->geometric_jacobian(joint_pos);
    } else {
        std::ostringstream message;
        message << "Invalid Cartesian force size: expected " << (task_count > 0 ? task_count : 6) << " or 6, got "
                << cart_force.size();
        throw std::invalid_argument(message.str());
    }

    if (force_jacobian.rows() != cart_force.size() || force_jacobian.cols() != joint_count) {
        throw std::runtime_error("Model returned an invalid Jacobian size");
    }
    joint_torque += force_jacobian.transpose() * cart_force;
    return joint_torque;
}

Eigen::VectorXd ArmSolve::static_force(Eigen::VectorXd joint_pos, Eigen::VectorXd joint_torque_residual) {
    if (joint_torque_residual.size() != model_->dof()) {
        std::ostringstream message;
        message << "Invalid joint torque residual size: expected " << model_->dof() << ", got "
                << joint_torque_residual.size();
        throw std::invalid_argument(message.str());
    }

    Eigen::MatrixXd jacobian;
    if (task_.empty()) {
        jacobian = model_->geometric_jacobian(joint_pos);
    } else {
        jacobian = ik_solver_.jacobian(joint_pos, task_);
    }

    if (jacobian.cols() != model_->dof()) {
        throw std::runtime_error("Model returned an invalid Jacobian size");
    }

    const Eigen::MatrixXd joint_to_task_jacobian = jacobian.transpose();
    return joint_to_task_jacobian.completeOrthogonalDecomposition().solve(joint_torque_residual);
}
