#pragma once

#include <Eigen/Dense>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <vector>

class Point {
public:
    Point()
        : time(std::chrono::nanoseconds(0)) {
    }

    explicit Point(int dof)
        : time(std::chrono::nanoseconds(0)) {
            pos.resize(dof);
            vel.resize(dof);
            acc.resize(dof);
    };

    Point(const rclcpp::Duration& time_from_start,
          const Eigen::VectorXd& position,
          const Eigen::VectorXd& velocity,
          const Eigen::VectorXd& acceleration)
        : time(time_from_start)
        , pos(position)
        , vel(velocity)
        , acc(acceleration) {
    }

    rclcpp::Duration time;
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
};

class Trajectory {
public:
    Trajectory(int dof,int max_point_num);
    bool add_point(const Point& point, rclcpp::Duration time_from_start);

    void start(rclcpp::Time time);
    void stop();
    void update(rclcpp::Time time, Point& point);
    void clear();
    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::size_t capacity() const;
    [[nodiscard]] bool empty() const;
    [[nodiscard]] const Point& front() const;
    [[nodiscard]] const Point& back() const;
    std::vector<Point> points;

private:
    // 在归一化参数 s ∈ [0,1] 处采样 a -> b 之间的五次贝塞尔曲线段（等价于
    // 五次 Hermite 插值），匹配端点位置、速度、加速度（缺失时按零处理），
    // 从而保证相邻段速度与加速度连续。
    void sample_segment(const Point& a, const Point& b, double s, Point* out) const;

    rclcpp::Time start_time_point_;
    bool started_ = false;
    Point copy;
    std::size_t index{0};
};
