#pragma once

#include "modelbase.hpp"
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

class ModelFromURDF : public ModelBase{
public:
    ModelFromURDF();
    int dof() const override;
    Eigen::Isometry3d forward_kinematics(const Eigen::VectorXd& q) const override;
    Eigen::MatrixXd geometric_jacobian(const Eigen::VectorXd& q) const override; 
    Eigen::VectorXd lower_jointLimit() const override; 
    Eigen::VectorXd upper_jointLimit() const override;
private:
};
