#include "model_from_urdf.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <filesystem>
#include <sstream>
#include <stdexcept>


ModelFromURDF::ModelFromURDF(const std::string& file_name, const std::string& end_link_name) {
    std::filesystem::path urdf_file(file_name);
    if (!std::filesystem::is_regular_file(file_name)) {
        throw std::runtime_error("Unable to find the robot URDF at: " + urdf_file.string());
    }

    pinocchio::urdf::buildModel(urdf_file.string(), model_);
    if (!model_.existFrame(end_link_name)) {
        throw std::runtime_error("The URDF model does not contain the end-effector frame: " + std::string(end_link_name));
    }

    end_effector_frame_id_ = model_.getFrameId(end_link_name);
}

int ModelFromURDF::dof() const {
    return model_.nv;
}

Eigen::Isometry3d ModelFromURDF::forward_kinematics(const Eigen::VectorXd& q) const {
    check_vector_dimension(q);
    pinocchio::framesForwardKinematics(model_, data_, q);

    const pinocchio::SE3& end_effector_pose = data_.oMf[end_effector_frame_id_];
    Eigen::Isometry3d pose                  = Eigen::Isometry3d::Identity();
    pose.linear()                           = end_effector_pose.rotation();
    pose.translation()                      = end_effector_pose.translation();
    return pose;
}

Eigen::MatrixXd ModelFromURDF::geometric_jacobian(const Eigen::VectorXd& q) const {
    check_vector_dimension(q);

    Eigen::MatrixXd jacobian(6, model_.nv);
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    return jacobian;
}

Eigen::VectorXd ModelFromURDF::lower_jointLimit() const {
    return model_.lowerPositionLimit;
}

Eigen::VectorXd ModelFromURDF::upper_jointLimit() const {
    return model_.upperPositionLimit;
}

pinocchio::Model ModelFromURDF::load_model() {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(locate_urdf(), model);
    return model;
}


void ModelFromURDF::check_vector_dimension(const Eigen::VectorXd& q) const {
    if (q.size() != model_.nq) {
        std::ostringstream message;
        message << "Invalid configuration size: expected " << model_.nq << ", got " << q.size();
        throw std::invalid_argument(message.str());
    }
}
