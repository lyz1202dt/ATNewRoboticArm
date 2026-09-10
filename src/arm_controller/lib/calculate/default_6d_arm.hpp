#pragma once

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

#include <string>

#include "robotic_arm.hpp"


class Default6DArm : public RoboticArm{
public:
    static constexpr int kJointCount = 6;

    Default6DArm(const std::string &urdf_file, const std::string &end_link_name);

    bool forward_kinmaic(const Eigen::VectorXd &joint_pos,
                         const Eigen::VectorXd &joint_vel,
                         const Eigen::VectorXd &joint_acc,
                         Eigen::VectorXd *cart_pos,
                         Eigen::VectorXd *cart_vel,
                         Eigen::VectorXd *cart_acc) override;

    bool inverse_kinmaic(const Eigen::VectorXd &cart_pos,
                         const Eigen::VectorXd &cart_vel,
                         const Eigen::VectorXd &cart_acc,
                         Eigen::VectorXd *joint_pos,
                         Eigen::VectorXd *joint_vel,
                         Eigen::VectorXd *joint_acc) override;

    bool forward_dynamic(const Eigen::VectorXd &joint_pos,
                         const Eigen::VectorXd &joint_vel,
                         const Eigen::VectorXd &joint_acc,
                         const Eigen::VectorXd &end_force,
                         Eigen::VectorXd *joint_torque) override;

    bool inverse_dynamic(const Eigen::VectorXd &joint_pos,
                         const Eigen::VectorXd &joint_vel,
                         const Eigen::VectorXd &joint_acc,
                         const Eigen::VectorXd &joint_torque,
                         Eigen::VectorXd *end_force) override;

private:
    KDL::Chain chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jac_solver_;
    KDL::ChainJntToJacDotSolver jdot_solver_;
    KDL::ChainIkSolverVel_pinv vel_solver_;
    KDL::ChainIkSolverPos_LMA ik_solver_;
    KDL::ChainDynParam dyn_solver_;

    KDL::JntSpaceInertiaMatrix mass_matrix_;
    KDL::JntArray coriolis_;
    KDL::JntArray gravity_;
    KDL::Jacobian jacobian_cache_;
    KDL::JntArrayVel joint_vel_cache_;
    KDL::JntArray ik_seed_cache_;
    KDL::JntArray last_joint_solution_;
    KDL::Twist jdot_qdot_cache_;
};
