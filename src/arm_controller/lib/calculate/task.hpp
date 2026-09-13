#pragma once

#include <Eigen/Dense>

class TaskMapping {
public:
    virtual ~TaskMapping() = default;

    // Map the model state to the user-defined task coordinates.
    virtual bool position_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::Isometry3d& pose,
                              Eigen::VectorXd* task_position) = 0;

    // Map the model geometric Jacobian to the Jacobian of those coordinates.
    virtual bool jacobian_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::MatrixXd& jacobian,
                              Eigen::MatrixXd* task_jacobian) = 0;
};
