#include "kinamic.hpp"

#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>
#include <stdexcept>

IKSolver::IKSolver(ModelBase* robot, TaskMapping* task_mapping)
    : robot_(robot), task_mapping_(task_mapping) {
    if (robot_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a model");
    }
    if (task_mapping_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a TaskMapping");
    }
}

Eigen::MatrixXd IKSolver::jacobian(const Eigen::VectorXd& q) {
    const Eigen::MatrixXd geometric_jacobian = robot_->geometric_jacobian(q);
    Eigen::MatrixXd task_jacobian;

    if (!task_mapping_->jacobian_map(q, geometric_jacobian, &task_jacobian)) {
        return {};
    }

    if (task_jacobian.cols() != geometric_jacobian.cols() || !task_jacobian.allFinite()) {
        return {};
    }

    return task_jacobian;
}

IKResult IKSolver::solve(const IKProblem& problem) {
    IKResult result;

    const std::vector<TaskUnit>& components = problem.task;
    const int m = static_cast<int>(components.size());
    const int n = robot_->dof();

    if (m <= 0 || problem.initial_q.size() != n) {
        return result;
    }

    // Joint limits.
    const Eigen::VectorXd lower = robot_->lower_jointLimit();
    const Eigen::VectorXd upper = robot_->upper_jointLimit();
    const bool use_joint_limits = problem.enable_joint_limits && lower.size() == n && upper.size() == n;

    // Per-joint step limits (a single scalar is applied to every joint).
    const bool step_per_joint = problem.enable_step_limits && problem.max_step.size() == n;
    const bool step_scalar    = problem.enable_step_limits && problem.max_step.size() == 1;

    Eigen::VectorXd q = problem.initial_q;
    if (use_joint_limits) {
        q = q.cwiseMax(lower).cwiseMin(upper);
    }

    Eigen::VectorXd target(m);
    Eigen::VectorXd weights(m);
    for (int i = 0; i < m; ++i) {
        target[i]  = components[i].target;
        weights[i] = components[i].weight;
    }

    auto compute_error = [&](const Eigen::VectorXd& qq, Eigen::VectorXd& error) {
        const Eigen::Isometry3d pose = robot_->forward_kinematics(qq);
        error.resize(m);
        for (int i = 0; i < m; ++i) {
            const TaskUnit& component = components[static_cast<std::size_t>(i)];
            double current = 0.0;
            switch (component.type) {
            case TaskUnit::PositionX:
            case TaskUnit::PositionY:
            case TaskUnit::PositionZ:
                current = pose.translation()[static_cast<Eigen::Index>(component.type)];
                break;
            case TaskUnit::AxisXX:
            case TaskUnit::AxisYY:
            case TaskUnit::AxisZZ: {
                constexpr double kAxisEpsilon = 1e-12;
                const Eigen::Index axis_index =
                    static_cast<Eigen::Index>(component.type) - TaskUnit::AxisXX;
                const double reference_norm = component.reference_axis.norm();
                if (reference_norm <= kAxisEpsilon) {
                    current = 0.0;
                    break;
                }
                current = component.reference_axis.normalized().dot(pose.rotation().col(axis_index));
                break;
            }
            default: return false;
            }
            error[i] = target[i] - current;
        }
        return error.allFinite();
    };

    constexpr int kMaxIterations = 200;
    constexpr double kTolerance  = 1e-6;
    constexpr double kDamping    = 1e-3;

    // A = I: the joint-increment box constraints are decoupled per joint.
    Eigen::SparseMatrix<double> constraint_matrix(n, n);
    constraint_matrix.setIdentity();

    Eigen::VectorXd error;
    for (int iter = 0; iter < kMaxIterations; ++iter) {
        result.iterations = iter + 1;

        if (!compute_error(q, error)) {
            return result;
        }
        const double error_norm = error.norm();
        result.error_norm       = error_norm;
        if (error_norm < kTolerance) {
            result.success = true;
            result.q       = q;
            return result;
        }

        // Weighted task Jacobian and residual.
        const Eigen::MatrixXd J = jacobian(q);
        if (J.rows() != m || J.cols() != n) {
            return result;
        }
        const Eigen::MatrixXd Jw = weights.asDiagonal() * J;
        const Eigen::VectorXd ew = weights.cwiseProduct(error);

        // QP cost: 0.5 * dq^T H dq + g^T dq, with
        //   H = 2 * (Jw^T Jw + lambda^2 I),  g = -2 * Jw^T ew,
        // i.e. min ||Jw dq - ew||^2 + lambda^2 ||dq||^2.
        const Eigen::MatrixXd H_dense       = 2.0 * (Jw.transpose() * Jw + kDamping * kDamping * Eigen::MatrixXd::Identity(n, n));
        Eigen::VectorXd gradient            = -2.0 * Jw.transpose() * ew;
        const Eigen::SparseMatrix<double> H = H_dense.sparseView();

        // Box constraints on dq: lower - q <= dq <= upper - q, tightened by step limits.
        Eigen::VectorXd lower_bound(n);
        Eigen::VectorXd upper_bound(n);
        for (int j = 0; j < n; ++j) {
            double lo = -OsqpEigen::INFTY;
            double hi = OsqpEigen::INFTY;
            if (use_joint_limits) {
                lo = lower[j] - q[j];
                hi = upper[j] - q[j];
            }
            if (step_per_joint) {
                lo = std::max(lo, -problem.max_step[j]);
                hi = std::min(hi, problem.max_step[j]);
            } else if (step_scalar) {
                lo = std::max(lo, -problem.max_step[0]);
                hi = std::min(hi, problem.max_step[0]);
            }
            lower_bound[j] = lo;
            upper_bound[j] = hi;
        }

        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n);
        solver.data()->setNumberOfConstraints(n);
        if (!solver.data()->setHessianMatrix(H) || !solver.data()->setGradient(gradient)
            || !solver.data()->setLinearConstraintsMatrix(constraint_matrix) || !solver.data()->setLowerBound(lower_bound)
            || !solver.data()->setUpperBound(upper_bound) || !solver.initSolver()) {
            return result;
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            return result;
        }

        const Eigen::VectorXd dq = solver.getSolution();
        q += dq;
    }

    if (!compute_error(q, error)) {
        return result;
    }
    result.q          = q;
    result.error_norm = error.norm();
    result.success    = result.error_norm < kTolerance;
    return result;
}
