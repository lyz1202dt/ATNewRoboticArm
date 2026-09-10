#include "default_6d_arm.hpp"

#include <cmath>
#include <stdexcept>

#include <kdl_parser/kdl_parser.hpp>

namespace {

KDL::Chain BuildChainOrThrow(const std::string &urdf_file, const std::string &end_link_name) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_file, tree)) {
        throw std::runtime_error("failed to parse URDF file: " + urdf_file);
    }

    const auto root_segment = tree.getRootSegment();
    if (root_segment == tree.getSegments().end()) {
        throw std::runtime_error("failed to find root segment in URDF file: " + urdf_file);
    }

    KDL::Chain chain;
    if (!tree.getChain(root_segment->first, end_link_name, chain)) {
        throw std::runtime_error("failed to build KDL chain from " + root_segment->first + " to " + end_link_name);
    }

    return chain;
}

}  // namespace

Default6DArm::Default6DArm(const std::string &urdf_file, const std::string &end_link_name)
    : chain_(BuildChainOrThrow(urdf_file, end_link_name)),
      fk_solver_(chain_),
      jac_solver_(chain_),
      jdot_solver_(chain_),
      vel_solver_(chain_, 1e-6, 150),
      ik_solver_(chain_, Eigen::Matrix<double, 6, 1>::Ones(), 1e-6, 200, 1e-12),
      dyn_solver_(chain_, KDL::Vector(0.0, 0.0, -9.81)),
      mass_matrix_(kJointCount),
      coriolis_(kJointCount),
      gravity_(kJointCount),
      jacobian_cache_(kJointCount),
      joint_vel_cache_(kJointCount),
      ik_seed_cache_(kJointCount),
      last_joint_solution_(kJointCount) {
    if (static_cast<int>(chain_.getNrOfJoints()) != kJointCount) {
        throw std::runtime_error("Default6DArm expects exactly 6 joints");
    }

    jdot_solver_.setHybridRepresentation();

    for (int i = 0; i < kJointCount; ++i) {
        last_joint_solution_(static_cast<unsigned int>(i)) = 0.0;
        ik_seed_cache_(static_cast<unsigned int>(i)) = 0.0;
    }
}

bool Default6DArm::forward_kinmaic(const Eigen::VectorXd &joint_pos,
                                   const Eigen::VectorXd &joint_vel,
                                   const Eigen::VectorXd &joint_acc,
                                   Eigen::VectorXd *cart_pos,
                                   Eigen::VectorXd *cart_vel,
                                   Eigen::VectorXd *cart_acc) {
    if (cart_pos == nullptr || cart_vel == nullptr || cart_acc == nullptr) {
        return false;
    }
    if (joint_pos.size() != kJointCount || joint_vel.size() != kJointCount || joint_acc.size() != kJointCount ||
        cart_pos->size() != kJointCount || cart_vel->size() != kJointCount || cart_acc->size() != kJointCount) {
        return false;
    }

    for (int i = 0; i < kJointCount; ++i) {
        const unsigned int idx = static_cast<unsigned int>(i);
        joint_vel_cache_.q(idx) = joint_pos(i);
        joint_vel_cache_.qdot(idx) = joint_vel(i);
    }

    KDL::Frame frame;
    if (fk_solver_.JntToCart(joint_vel_cache_.q, frame) < 0) {
        return false;
    }

    (*cart_pos)(0) = frame.p.x();
    (*cart_pos)(1) = frame.p.y();
    (*cart_pos)(2) = frame.p.z();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    frame.M.GetRPY(roll, pitch, yaw);
    (*cart_pos)(3) = roll;
    (*cart_pos)(4) = pitch;
    (*cart_pos)(5) = yaw;

    if (jac_solver_.JntToJac(joint_vel_cache_.q, jacobian_cache_) < 0) {
        return false;
    }

    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    Eigen::Matrix<double, 6, 1> qdot;
    Eigen::Matrix<double, 6, 1> qdd;
    for (int i = 0; i < kJointCount; ++i) {
        qdot(i) = joint_vel(i);
        qdd(i) = joint_acc(i);
    }

    const Eigen::Matrix<double, 6, 1> twist = jacobian * qdot;

    if (jdot_solver_.JntToJacDot(joint_vel_cache_, jdot_qdot_cache_) < 0) {
        return false;
    }

    Eigen::Matrix<double, 6, 1> jdot_qdot;
    jdot_qdot << jdot_qdot_cache_.vel.x(),
                 jdot_qdot_cache_.vel.y(),
                 jdot_qdot_cache_.vel.z(),
                 jdot_qdot_cache_.rot.x(),
                 jdot_qdot_cache_.rot.y(),
                 jdot_qdot_cache_.rot.z();

    const Eigen::Matrix<double, 6, 1> acc = jacobian * qdd + jdot_qdot;

    for (int i = 0; i < kJointCount; ++i) {
        (*cart_vel)(i) = twist(i);
        (*cart_acc)(i) = acc(i);
    }

    return true;
}

bool Default6DArm::inverse_kinmaic(const Eigen::VectorXd &cart_pos,
                                   const Eigen::VectorXd &cart_vel,
                                   const Eigen::VectorXd &cart_acc,
                                   Eigen::VectorXd *joint_pos,
                                   Eigen::VectorXd *joint_vel,
                                   Eigen::VectorXd *joint_acc) {
    if (joint_pos == nullptr || joint_vel == nullptr || joint_acc == nullptr) {
        return false;
    }
    if (cart_pos.size() != kJointCount || cart_vel.size() != kJointCount || cart_acc.size() != kJointCount ||
        joint_pos->size() != kJointCount || joint_vel->size() != kJointCount || joint_acc->size() != kJointCount) {
        return false;
    }

    KDL::Frame target(
        KDL::Rotation::RPY(cart_pos(3), cart_pos(4), cart_pos(5)),
        KDL::Vector(cart_pos(0), cart_pos(1), cart_pos(2)));

    for (int i = 0; i < kJointCount; ++i) {
        ik_seed_cache_(static_cast<unsigned int>(i)) = last_joint_solution_(static_cast<unsigned int>(i));
    }

    const int ik_result = ik_solver_.CartToJnt(ik_seed_cache_, target, last_joint_solution_);
    if (ik_result < 0) {
        for (int i = 0; i < kJointCount; ++i) {
            last_joint_solution_(static_cast<unsigned int>(i)) = ik_seed_cache_(static_cast<unsigned int>(i));
        }
    }

    for (int i = 0; i < kJointCount; ++i) {
        (*joint_pos)(i) = last_joint_solution_(static_cast<unsigned int>(i));
    }
    if (ik_result < 0) {
        for (int i = 0; i < kJointCount; ++i) {
            (*joint_vel)(i) = 0.0;
            (*joint_acc)(i) = 0.0;
        }
        return false;
    }

    for (int i = 0; i < kJointCount; ++i) {
        const unsigned int idx = static_cast<unsigned int>(i);
        joint_vel_cache_.q(idx) = last_joint_solution_(idx);
    }

    KDL::Twist cart_twist;
    cart_twist.vel = KDL::Vector(cart_vel(0), cart_vel(1), cart_vel(2));
    cart_twist.rot = KDL::Vector(cart_vel(3), cart_vel(4), cart_vel(5));
    int vel_result = vel_solver_.CartToJnt(joint_vel_cache_.q, cart_twist, joint_vel_cache_.qdot);
    if (vel_result < 0) {
        for (int i = 0; i < kJointCount; ++i) {
            (*joint_vel)(i) = 0.0;
            (*joint_acc)(i) = 0.0;
        }
        return false;
    }

    for (int i = 0; i < kJointCount; ++i) {
        (*joint_vel)(i) = joint_vel_cache_.qdot(static_cast<unsigned int>(i));
    }

    if (jdot_solver_.JntToJacDot(joint_vel_cache_, jdot_qdot_cache_) < 0) {
        return false;
    }

    KDL::Twist cart_acc_residual;
    cart_acc_residual.vel = KDL::Vector(cart_acc(0) - jdot_qdot_cache_.vel.x(),
                                        cart_acc(1) - jdot_qdot_cache_.vel.y(),
                                        cart_acc(2) - jdot_qdot_cache_.vel.z());
    cart_acc_residual.rot = KDL::Vector(cart_acc(3) - jdot_qdot_cache_.rot.x(),
                                        cart_acc(4) - jdot_qdot_cache_.rot.y(),
                                        cart_acc(5) - jdot_qdot_cache_.rot.z());

    vel_result = vel_solver_.CartToJnt(joint_vel_cache_.q, cart_acc_residual, joint_vel_cache_.qdot);
    if (vel_result < 0) {
        for (int i = 0; i < kJointCount; ++i) {
            (*joint_acc)(i) = 0.0;
        }
        return false;
    }

    for (int i = 0; i < kJointCount; ++i) {
        (*joint_acc)(i) = joint_vel_cache_.qdot(static_cast<unsigned int>(i));
    }

    return true;
}

bool Default6DArm::forward_dynamic(const Eigen::VectorXd &joint_pos,
                                   const Eigen::VectorXd &joint_vel,
                                   const Eigen::VectorXd &joint_acc,
                                   const Eigen::VectorXd &end_force,
                                   Eigen::VectorXd *joint_torque) {
    if (joint_torque == nullptr) {
        return false;
    }
    if (joint_pos.size() != kJointCount || joint_vel.size() != kJointCount || joint_acc.size() != kJointCount ||
        end_force.size() != kJointCount || joint_torque->size() != kJointCount) {
        return false;
    }

    Eigen::Matrix<double, 6, 1> qdd;
    Eigen::Matrix<double, 6, 1> wrench;
    for (int i = 0; i < kJointCount; ++i) {
        const unsigned int idx = static_cast<unsigned int>(i);
        joint_vel_cache_.q(idx) = joint_pos(i);
        joint_vel_cache_.qdot(idx) = joint_vel(i);
        qdd(i) = joint_acc(i);
        wrench(i) = end_force(i);
    }

    if (dyn_solver_.JntToMass(joint_vel_cache_.q, mass_matrix_) < 0 ||
        dyn_solver_.JntToCoriolis(joint_vel_cache_.q, joint_vel_cache_.qdot, coriolis_) < 0 ||
        dyn_solver_.JntToGravity(joint_vel_cache_.q, gravity_) < 0 ||
        jac_solver_.JntToJac(joint_vel_cache_.q, jacobian_cache_) < 0) {
        return false;
    }

    Eigen::Matrix<double, 6, 6> mass = mass_matrix_.data;
    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    Eigen::Matrix<double, 6, 1> coriolis;
    Eigen::Matrix<double, 6, 1> gravity;
    for (int i = 0; i < kJointCount; ++i) {
        coriolis(i) = coriolis_(static_cast<unsigned int>(i));
        gravity(i) = gravity_(static_cast<unsigned int>(i));
    }

    const Eigen::Matrix<double, 6, 1> torque = mass * qdd + coriolis + gravity + jacobian.transpose() * wrench;
    for (int i = 0; i < kJointCount; ++i) {
        (*joint_torque)(i) = torque(i);
    }

    return true;
}

bool Default6DArm::inverse_dynamic(const Eigen::VectorXd &joint_pos,
                                   const Eigen::VectorXd &joint_vel,
                                   const Eigen::VectorXd &joint_acc,
                                   const Eigen::VectorXd &joint_torque,
                                   Eigen::VectorXd *end_force) {
    if (end_force == nullptr) {
        return false;
    }
    if (joint_pos.size() != kJointCount || joint_vel.size() != kJointCount || joint_acc.size() != kJointCount ||
        joint_torque.size() != kJointCount || end_force->size() != kJointCount) {
        return false;
    }

    Eigen::Matrix<double, 6, 1> qdd;
    Eigen::Matrix<double, 6, 1> torque;
    for (int i = 0; i < kJointCount; ++i) {
        const unsigned int idx = static_cast<unsigned int>(i);
        joint_vel_cache_.q(idx) = joint_pos(i);
        joint_vel_cache_.qdot(idx) = joint_vel(i);
        qdd(i) = joint_acc(i);
        torque(i) = joint_torque(i);
    }

    if (dyn_solver_.JntToMass(joint_vel_cache_.q, mass_matrix_) < 0 ||
        dyn_solver_.JntToCoriolis(joint_vel_cache_.q, joint_vel_cache_.qdot, coriolis_) < 0 ||
        dyn_solver_.JntToGravity(joint_vel_cache_.q, gravity_) < 0 ||
        jac_solver_.JntToJac(joint_vel_cache_.q, jacobian_cache_) < 0) {
        return false;
    }

    Eigen::Matrix<double, 6, 6> mass = mass_matrix_.data;
    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    Eigen::Matrix<double, 6, 1> coriolis;
    Eigen::Matrix<double, 6, 1> gravity;
    for (int i = 0; i < kJointCount; ++i) {
        coriolis(i) = coriolis_(static_cast<unsigned int>(i));
        gravity(i) = gravity_(static_cast<unsigned int>(i));
    }

    const Eigen::Matrix<double, 6, 1> residual = torque - mass * qdd - coriolis - gravity;
    const Eigen::Matrix<double, 6, 1> force = jacobian.transpose().partialPivLu().solve(residual);

    for (int i = 0; i < kJointCount; ++i) {
        (*end_force)(i) = force(i);
    }

    return true;
}
