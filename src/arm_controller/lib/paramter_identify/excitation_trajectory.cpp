#include "excitation_trajectory.hpp"


ExcitationTrajectory::ExcitationTrajectory(const std::string& urdf_path) {
}
void ExcitationTrajectory::generate(
    rclcpp::Duration trajectory_time, int repeat_cnt, float joint_vel_limit_scale, float joint_pos_limit_scale) {
}
bool ExcitationTrajectory::generate_is_finished() {
}

bool ExcitationTrajectory::get_target_position(rclcpp::Duration time, Eigen::VectorXd& pos) {
}

void calc_traj()
{

}