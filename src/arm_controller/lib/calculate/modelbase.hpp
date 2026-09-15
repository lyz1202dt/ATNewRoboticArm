#pragma once

#include <Eigen/Dense>

class ModelBase{
public:
    ModelBase() = default;
    virtual ~ModelBase() = default;
    virtual int dof() const = 0;
    virtual Eigen::Isometry3d forward_kinematics(const Eigen::VectorXd& q) const = 0;
    virtual bool geometric_jacobian(const Eigen::VectorXd& q, Eigen::MatrixXd* jacobian) const = 0;

    virtual bool inverse_dynamic(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq,
                                 Eigen::VectorXd* joint_torque) = 0;

    virtual const Eigen::VectorXd& lower_jointLimit() const = 0; 
    virtual const Eigen::VectorXd& upper_jointLimit() const = 0;
};
