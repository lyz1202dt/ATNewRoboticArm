#pragma once

#include <string>
#include <Eigen/Dense>


class RoboticArm{
public:
    RoboticArm() = default;
    virtual ~RoboticArm() = default;

    virtual bool forward_kinmaic(const Eigen::VectorXd &joint_pos,
    const Eigen::VectorXd &joint_vel,
    const Eigen::VectorXd &joint_acc,
    Eigen::VectorXd *cart_pos,
    Eigen::VectorXd *cart_vel,
    Eigen::VectorXd *cart_acc)=0;
    virtual bool inverse_kinmaic(const Eigen::VectorXd &cart_pos,
    const Eigen::VectorXd &cart_vel,
    const Eigen::VectorXd &cart_acc,
    Eigen::VectorXd *joint_pos,
    Eigen::VectorXd *joint_vel,
    Eigen::VectorXd *joint_acc)=0;
    virtual bool forward_dynamic(const Eigen::VectorXd &joint_pos,
    const Eigen::VectorXd &joint_vel,
    const Eigen::VectorXd &joint_acc,
    const Eigen::VectorXd &end_force,
    Eigen::VectorXd *joint_torque)=0;
    virtual bool inverse_dynamic(const Eigen::VectorXd &joint_pos,
    const Eigen::VectorXd &joint_vel,
    const Eigen::VectorXd &joint_acc,
    const Eigen::VectorXd &joint_torque,
    Eigen::VectorXd *end_force)=0;
};
