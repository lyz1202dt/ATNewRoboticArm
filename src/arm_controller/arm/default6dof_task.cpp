#include "default6dof_task.hpp"

#include <Eigen/Geometry>

#include <cmath>

Default6DofTaskSpaceMapping::Default6DofTaskSpaceMapping() = default;

bool Default6DofTaskSpaceMapping::position_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::Isometry3d& pose,
                                               Eigen::VectorXd* task_position) {
    (void)joint_pos;
    if (task_position == nullptr) {
        return false;
    }

    if (task_position->size() != 6) {
        task_position->resize(6);
    }
    task_position->head<3>() = pose.translation();

    Eigen::AngleAxisd rotation(pose.linear());
    if (std::abs(rotation.angle()) < 1e-12) {
        task_position->tail<3>().setZero();
    } else {
        task_position->tail<3>() = rotation.angle() * rotation.axis();
    }

    return task_position->allFinite();
}

bool Default6DofTaskSpaceMapping::jacobian_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::MatrixXd& jacobian,
                                               Eigen::MatrixXd* task_jacobian) {
    (void)joint_pos;
    if (task_jacobian == nullptr || jacobian.rows() != 6) {
        return false;
    }

    if (task_jacobian->rows() != jacobian.rows() || task_jacobian->cols() != jacobian.cols()) {
        task_jacobian->resize(jacobian.rows(), jacobian.cols());
    }
    task_jacobian->noalias() = jacobian;
    return task_jacobian->allFinite();
}
