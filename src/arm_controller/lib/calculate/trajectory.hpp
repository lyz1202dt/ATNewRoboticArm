#pragma once

#include <Eigen/Dense>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <vector>

class Point{
public:
    rclcpp::Duration time;
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
};

class Trajectory{
public:
    Trajectory();
    void add_point(const Point &point,rclcpp::Duration time_from_start);

    void start(rclcpp::Time time);
    void stop();
    void update(rclcpp::Time time,Point& point);
    std::vector<Point> points;
private:
    // 在归一化参数 s ∈ [0,1] 处采样 a -> b 之间的五次贝塞尔曲线段（等价于
    // 五次 Hermite 插值），匹配端点位置、速度、加速度（缺失时按零处理），
    // 从而保证相邻段速度与加速度连续。
    void sample_segment(const Point& a, const Point& b, double s, Point* out) const;

    rclcpp::Time start_time_point_;
    bool started_ = false;
};

inline Trajectory operator+(const Trajectory& traj,Point point)
{
    Trajectory new_traj=traj;
    point.time=traj.points.back().time+point.time;
    new_traj.points.emplace_back(point);
    return new_traj;
}
