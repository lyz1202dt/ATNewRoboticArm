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

    Eigen::VectorXd position(6);
    position.head<3>() = pose.translation();

    Eigen::AngleAxisd rotation(pose.linear());
    if (std::abs(rotation.angle()) < 1e-12) {
        position.tail<3>().setZero();
    } else {
        position.tail<3>() = rotation.angle() * rotation.axis();
    }

    *task_position = position;
    return task_position->allFinite();
}

bool Default6DofTaskSpaceMapping::jacobian_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::MatrixXd& jacobian,
                                               Eigen::MatrixXd* task_jacobian) {
    (void)joint_pos;
    if (task_jacobian == nullptr || jacobian.rows() != 6) {
        return false;
    }

    *task_jacobian = jacobian;
    return task_jacobian->allFinite();
}
