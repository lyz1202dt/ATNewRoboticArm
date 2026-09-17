#pragma once

#include <string>
#include <Eigen/Dense>

class ParamterIdentify{
public:
    ParamterIdentify(const std::string &urdf_path);

    //参数数量，自由度；预先resize内存，防止运行时分配内存
    void application_memory(int param_num,int dof);

    //返回剩余的可记录元素数量
    size_t record_value(const Eigen::VectorXd &pos,const Eigen::VectorXd &vel,const Eigen::VectorXd &torque);

    //对vel进行滤波，得到acc估计值
    bool estimate_acc();
private:
    std::vector<Eigen::VectorXd> pos;
    std::vector<Eigen::VectorXd> vel;
    std::vector<Eigen::VectorXd> acc;
    std::vector<Eigen::VectorXd> torque;
};