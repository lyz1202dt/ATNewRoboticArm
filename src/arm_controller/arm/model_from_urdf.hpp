#pragma once

#include "modelbase.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

class ModelFromURDF : public ModelBase{
public:
    explicit ModelFromURDF(const std::string& file_name,const std::string &end_link_name);

    int dof() const override;
    Eigen::Isometry3d forward_kinematics(const Eigen::VectorXd& q) const override;
    bool geometric_jacobian(const Eigen::VectorXd& q, Eigen::MatrixXd* jacobian) const override;
    bool inverse_dynamic(const Eigen::VectorXd& q,
                         const Eigen::VectorXd& dq,
                         const Eigen::VectorXd& ddq,
                         Eigen::VectorXd* joint_torque) override;
    const Eigen::VectorXd& lower_jointLimit() const override; 
    const Eigen::VectorXd& upper_jointLimit() const override;

private:
    void check_vector_dimension(const Eigen::VectorXd& q) const;

    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    pinocchio::FrameIndex end_effector_frame_id_{0};
};
