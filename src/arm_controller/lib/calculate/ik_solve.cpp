#include "ik_solve.hpp"
#include <Eigen/Sparse>
#include <Eigen/src/Core/Matrix.h>
#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>
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

    Eigen::MatrixXd J_task6(6, J.cols());
    J_task6.topRows(3)    = J.topRows(3);
    J_task6.bottomRows(3) = J_rpy;

    Eigen::MatrixXd J_task(static_cast<Eigen::Index>(task.components().size()), J.cols());
    for (Eigen::Index i = 0; i < J_task.rows(); ++i) {
        J_task.row(i) = J_task6.row(static_cast<Eigen::Index>(task.components()[i].type));
    }

    return J_task;
}

IKResult IKSolver::solve(const IKProblem& problem) {
    IKResult result;

    const std::vector<TaskUnit>& components = problem.task.components();
    const int m = static_cast<int>(components.size());
    const int n = robot_->dof();

    if (m <= 0 || problem.initial_q.size() != n) {
        return result;
    }

    constexpr double kPi = 3.14159265358979323846;

    // Joint limits.
    const Eigen::VectorXd lower = robot_->lower_jointLimit();
    const Eigen::VectorXd upper = robot_->upper_jointLimit();
    const bool use_joint_limits =
        problem.enable_joint_limits && lower.size() == n && upper.size() == n;

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

    // Task-space coordinates follow the TaskUnit order: [x, y, z, roll, pitch, yaw].
    auto compute_error = [&](const Eigen::VectorXd& qq, Eigen::VectorXd& error) {
        const Eigen::Isometry3d pose = robot_->forward_kinematics(qq);
        const Eigen::Vector3d euler_zyx = pose.rotation().eulerAngles(2, 1, 0);  // [yaw, pitch, roll]

        Eigen::VectorXd current(6);
        current << pose.translation().x(), pose.translation().y(), pose.translation().z(),
                   euler_zyx[2], euler_zyx[1], euler_zyx[0];  // roll, pitch, yaw

        error.resize(m);
        for (int i = 0; i < m; ++i) {
            const int type = static_cast<int>(components[i].type);
            double e = target[i] - current[type];
            if (type >= TaskUnit::YAW) {
                e = std::remainder(e, 2.0 * kPi);
            }
            error[i] = e;
        }
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

        compute_error(q, error);
        const double error_norm = error.norm();
        result.error_norm = error_norm;
        if (error_norm < kTolerance) {
            result.success = true;
            result.q = q;
            return result;
        }

        // Weighted task Jacobian and residual.
        const Eigen::MatrixXd J  = jacobian(q, problem.task);
        const Eigen::MatrixXd Jw = weights.asDiagonal() * J;
        const Eigen::VectorXd ew = weights.cwiseProduct(error);

        // QP cost: 0.5 * dq^T H dq + g^T dq, with
        //   H = 2 * (Jw^T Jw + lambda^2 I),  g = -2 * Jw^T ew,
        // i.e. min ||Jw dq - ew||^2 + lambda^2 ||dq||^2.
        const Eigen::MatrixXd H_dense =
            2.0 * (Jw.transpose() * Jw + kDamping * kDamping * Eigen::MatrixXd::Identity(n, n));
        const Eigen::VectorXd gradient = -2.0 * Jw.transpose() * ew;
        const Eigen::SparseMatrix<double> H = H_dense.sparseView();

        // Box constraints on dq: lower - q <= dq <= upper - q, tightened by step limits.
        Eigen::VectorXd lower_bound(n);
        Eigen::VectorXd upper_bound(n);
        for (int j = 0; j < n; ++j) {
            double lo = -OsqpEigen::INFTY;
            double hi =  OsqpEigen::INFTY;
            if (use_joint_limits) {
                lo = lower[j] - q[j];
                hi = upper[j] - q[j];
            }
            if (step_per_joint) {
                lo = std::max(lo, -problem.max_step[j]);
                hi = std::min(hi,  problem.max_step[j]);
            } else if (step_scalar) {
                lo = std::max(lo, -problem.max_step[0]);
                hi = std::min(hi,  problem.max_step[0]);
            }
            lower_bound[j] = lo;
            upper_bound[j] = hi;
        }

        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n);
        solver.data()->setNumberOfConstraints(n);
        if (!solver.data()->setHessianMatrix(H) ||
            !solver.data()->setGradient(gradient) ||
            !solver.data()->setLinearConstraintsMatrix(constraint_matrix) ||
            !solver.data()->setLowerBound(lower_bound) ||
            !solver.data()->setUpperBound(upper_bound) ||
            !solver.initSolver()) {
            return result;
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            return result;
        }

        const Eigen::VectorXd dq = solver.getSolution();
        q += dq;
    }

    compute_error(q, error);
    result.q          = q;
    result.error_norm = error.norm();
    result.success    = result.error_norm < kTolerance;
    return result;
}
