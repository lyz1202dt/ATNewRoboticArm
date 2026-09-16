#include "excitation_trajectory.hpp"

#include <Eigen/SVD>

#include <Eigen/src/Core/Matrix.h>
#include <cmath>

ExcitationTrajectory::ExcitationTrajectory(const std::string& urdf_path) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    q.resize(model_.nq);
    dq.resize(model_.nv);
    ddq.resize(model_.nv);
}
void ExcitationTrajectory::generate(
    rclcpp::Duration trajectory_time, int repeat_cnt, float joint_vel_limit_scale, float joint_pos_limit_scale) {
    (void)trajectory_time;
    (void)repeat_cnt;
    (void)joint_vel_limit_scale;
    (void)joint_pos_limit_scale;

    calc_traj();
    trajectory_generatefinished = true;
}
bool ExcitationTrajectory::generate_is_finished() {
    return trajectory_generatefinished;
}

bool ExcitationTrajectory::get_target_position(rclcpp::Duration time, Eigen::VectorXd& pos) {
    (void)time;
    (void)pos;

    return false;
}

void ExcitationTrajectory::calc_traj() {

    constexpr int k_traj_cnt=400;

    // SVD 分解，评估最小参数集
    std::random_device rd;
    std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
    std::mt19937 random_genenter(seq);

    std::uniform_real_distribution<double> pos_range(svd_pos_lower, svd_pos_upper);
    std::uniform_real_distribution<double> vel_range(-svd_vel_upper, svd_vel_upper);
    std::uniform_real_distribution<double> acc_range(-svd_acc_upper, svd_acc_upper);

    // 填写随机参数
    for (int i = 0; i < svd_Y_stack_num; i++) {
        for (int j = 0; j < model_.nv; j++) {
            q[j] = pos_range(random_genenter);
            dq[j]  = vel_range(random_genenter);
            ddq[j] = acc_range(random_genenter);
        }

        const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
        if (i == 0) {
            Y.resize(Yi.rows() * svd_Y_stack_num, Yi.cols());
        }
        Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeFullV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    int identifiable_parameter_num_           = 0;
    for (int i = 0; i < singular_values.size(); i++) {
        if (singular_values[i] > SVD_ZERO) {
            identifiable_parameter_num_++;
        }
    }


    std::uniform_real_distribution<double> fourier_range(-2.0, 2.0);

    // 3次谐波，关节个数维度，一个周期（足够评价轨迹质量了）
    FourierTrajectory fourier_traj(model_.nq, 3, 4);
    Eigen::MatrixXd param_mat((model_.nq, 2 * fourier_traj.harmonics()));
    double max_socor=0.0;
    Eigen::MatrixXd best_mat;
    for (int i = 0; i < k_traj_cnt; i++) // 随机生成400个轨迹，并进行可行性检查，评分，使用评分最高的一个
    {
        // 随机化傅里叶系数矩阵：每行对应一个关节，列按 [a1,b1,a2,b2,a3,b3] 排列
        for (int r = 0; r < param_mat.rows(); ++r) {
            for (int c = 0; c < param_mat.cols(); ++c) {
                param_mat(r, c) = fourier_range(random_genenter);
            }
        }
        fourier_traj.set_coefficients(param_mat);
        if (!traj_is_available(fourier_traj, 0.005))
            continue;
        auto ret=traj_score(fourier_traj,0.005,identifiable_parameter_num_);
        std::cout<<"socor:"<<std::get<0>(ret)<<std::get<1>(ret)<<std::endl;
        if(std::get<0>(ret)*0.5+std::get<1>(ret)*0.5>max_socor)
        {
            max_socor=std::get<0>(ret)*0.5+std::get<1>(ret)*0.5;
            best_mat=param_mat;
        }
    }
    std::cout<<"best_mat"<<best_mat<<std::endl;
}

bool ExcitationTrajectory::traj_is_available(const FourierTrajectory& traj, double dt) {
    // 轨迹关节数与模型不一致时视为不可用
    if (traj.dof() != model_.nq) {
        return false;
    }

    // 傅里叶轨迹为周期轨迹，采样一个周期即可覆盖全部位置/速度取值
    const int sample_cnt = static_cast<int>(traj.period()/dt);

    for (int i = 0; i <= sample_cnt; ++i) {
        const double t           = i * dt;
        q  = traj.position(t);
        dq = traj.velocity(t);

        for (int j = 0; j < traj.dof(); ++j) {
            // 位置软限位：将关节行程按软限位比例缩放到行程中点附近
            const double lower      = model_.lowerPositionLimit[j];
            const double upper      = model_.upperPositionLimit[j];
            const double mid        = 0.5 * (lower + upper);
            const double half_range = 0.5 * (upper - lower);
            const double soft_lower = mid - traj_pos_soft_limit_rate * half_range;
            const double soft_upper = mid + traj_pos_soft_limit_rate * half_range;

            if (q[j] < soft_lower || q[j] > soft_upper) {
                return false;
            }

            // 关节速度超限
            if (std::fabs(dq[j]) > model_.velocityLimit[j]) {
                return false;
            }
        }
    }

    return true;
}

std::tuple<double,double>  ExcitationTrajectory::traj_score(const FourierTrajectory& traj, double dt,int identifiable_param_num) // 求激励轨迹的得分
{
    const int sample_cnt = static_cast<int>(traj.period() / dt);
    for (int i = 0; i < sample_cnt; i++) {
        q   = traj.position(i * dt);
        dq  = traj.velocity(i * dt);
        ddq = traj.acceleration(i * dt);

        const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
        Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
    }
    //进行SVD，对轨迹质量进行评分
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinV);
    S = svd.singularValues();

    double a1=S[0]/S[identifiable_param_num-1]; //条件数
    double a2=0.0f;
    for(int i=0;i<identifiable_param_num;i++)
        a2=a2+std::log10(S[i]);
    return std::make_tuple(a1,a2/identifiable_param_num);
}