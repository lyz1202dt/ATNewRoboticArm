#pragma once

#include <Eigen/Dense>

class TaskMapping {
public:
    virtual ~TaskMapping() = default;
    virtual bool jacobian_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::MatrixXd& jacobian,
                              Eigen::MatrixXd* task_jacobian) = 0;
};
