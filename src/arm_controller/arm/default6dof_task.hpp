#pragma once

#include "task.hpp"
class Default6DofTaskSpaceMapping : public TaskMapping{
public:
    Default6DofTaskSpaceMapping();

    bool position_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::Isometry3d& pose,
                      Eigen::VectorXd* task_position) override;

    bool jacobian_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::MatrixXd& jacobian,
                      Eigen::MatrixXd* task_jacobian) override;
};
