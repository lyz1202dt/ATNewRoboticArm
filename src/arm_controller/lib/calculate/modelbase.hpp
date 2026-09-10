#pragma once

#include <Eigen/Dense>

class ModelBase{
public:
    ModelBase();
    virtual ~ModelBase() = default;
    virtual int dof() const = 0;
    virtual Eigen::Isometry3d forward_kinematics(const Eigen::VectorXd& q) const = 0;
    virtual Eigen::MatrixXd geometric_jacobian(const Eigen::VectorXd& q) const = 0; 
    virtual Eigen::VectorXd lower_jointLimit() const = 0; 
    virtual Eigen::VectorXd upper_jointLimit() const = 0;
};

