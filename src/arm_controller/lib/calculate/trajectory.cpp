#include "trajectory.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace {

// 两点时间间隔过小视为无效段，避免除零。
constexpr double kMinSegmentDuration = 1e-9;

}  // namespace

Trajectory::Trajectory() = default;

void Trajectory::add_point(const Point& point, rclcpp::Duration time_from_start) {
    if (!points.empty() && point.pos.size() != points.front().pos.size()) {
        throw std::invalid_argument("Trajectory point position dimension mismatch");
    }

    Point copy = point;
    copy.time = time_from_start;

    // 按相对起点的时刻保持 points 单调有序，update 依赖该顺序。
    const auto insert_position = std::lower_bound(
        points.begin(), points.end(), copy,
        [](const Point& lhs, const Point& rhs) { return lhs.time < rhs.time; });
    points.insert(insert_position, std::move(copy));
}

void Trajectory::start(rclcpp::Time time) {
    start_time_point_ = time;
    started_ = true;
}

void Trajectory::stop() {
    started_ = false;
}

void Trajectory::update(rclcpp::Time time, Point& point) {
    if (!started_ || points.empty()) {
        return;
    }

    const rclcpp::Duration elapsed = time - start_time_point_;

    // 起始点之前与终止点之后保持端点（夹持）。
    if (elapsed <= points.front().time) {
        point = points.front();
        return;
    }
    if (elapsed >= points.back().time) {
        point = points.back();
        return;
    }

    // 找到包含 elapsed 的段 [segment, segment + 1]。
    size_t segment = 0;
    while (segment + 1 < points.size() && points[segment + 1].time <= elapsed) {
        ++segment;
    }

    const double start_seconds = points[segment].time.seconds();
    const double end_seconds = points[segment + 1].time.seconds();
    const double duration = end_seconds - start_seconds;
    if (duration <= kMinSegmentDuration) {
        point = points[segment];
        return;
    }

    const double s = (elapsed.seconds() - start_seconds) / duration;
    sample_segment(points[segment], points[segment + 1], s, &point);
    point.time = elapsed;
}

void Trajectory::sample_segment(const Point& a, const Point& b, double s, Point* out) const {
    const double duration = (b.time - a.time).seconds();

    const int dimension = a.pos.size();
    const auto value_or_zero = [dimension](const Eigen::VectorXd& v) {
        return v.size() == dimension ? v : Eigen::VectorXd::Zero(dimension);
    };

    // 端点约束：位置、速度、加速度（缺失时按零处理）。
    const Eigen::VectorXd p0 = a.pos;
    const Eigen::VectorXd p1 = b.pos;
    const Eigen::VectorXd v0 = value_or_zero(a.vel);
    const Eigen::VectorXd v1 = value_or_zero(b.vel);
    const Eigen::VectorXd a0 = value_or_zero(a.acc);
    const Eigen::VectorXd a1 = value_or_zero(b.acc);

    // 五次贝塞尔控制点（等价于五次 Hermite 插值）：
    // 使端点位置/速度/加速度等于给定值，从而保证相邻段速度与加速度连续。
    const double t = duration;
    const Eigen::VectorXd b0 = p0;
    const Eigen::VectorXd b1 = p0 + (t / 5.0) * v0;
    const Eigen::VectorXd b2 = p0 + (2.0 * t / 5.0) * v0 + (t * t / 20.0) * a0;
    const Eigen::VectorXd b3 = p1 - (2.0 * t / 5.0) * v1 + (t * t / 20.0) * a1;
    const Eigen::VectorXd b4 = p1 - (t / 5.0) * v1;
    const Eigen::VectorXd b5 = p1;

    // 五次 Bernstein 基函数。
    const double u = s;
    const double v = 1.0 - u;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double v2 = v * v;
    const double v3 = v2 * v;
    const double v4 = v3 * v;

    const double w0 = v4 * v;          // (1-u)^5
    const double w1 = 5.0 * u * v4;    // 5u(1-u)^4
    const double w2 = 10.0 * u2 * v3;  // 10u^2(1-u)^3
    const double w3 = 10.0 * u3 * v2;  // 10u^3(1-u)^2
    const double w4 = 5.0 * u4 * v;    // 5u^4(1-u)
    const double w5 = u4 * u;          // u^5

    out->pos = w0 * b0 + w1 * b1 + w2 * b2 + w3 * b3 + w4 * b4 + w5 * b5;

    // 一阶导（对 s）：B'(s) = 5 · 四次贝塞尔，控制点为 Δi = B_{i+1} - B_i。
    const double z0 = v4;             // (1-u)^4
    const double z1 = 4.0 * u * v3;   // 4u(1-u)^3
    const double z2 = 6.0 * u2 * v2;  // 6u^2(1-u)^2
    const double z3 = 4.0 * u3 * v;   // 4u^3(1-u)
    const double z4 = u4;             // u^4

    const Eigen::VectorXd d0 = b1 - b0;
    const Eigen::VectorXd d1 = b2 - b1;
    const Eigen::VectorXd d2 = b3 - b2;
    const Eigen::VectorXd d3 = b4 - b3;
    const Eigen::VectorXd d4 = b5 - b4;

    out->vel = 5.0 * (z0 * d0 + z1 * d1 + z2 * d2 + z3 * d3 + z4 * d4) / duration;

    // 二阶导（对 s）：B''(s) = 20 · 三次贝塞尔，控制点为 Δ_{i+1} - Δ_i。
    const double y0 = v3;           // (1-u)^3
    const double y1 = 3.0 * u * v2; // 3u(1-u)^2
    const double y2 = 3.0 * u2 * v; // 3u^2(1-u)
    const double y3 = u3;           // u^3

    out->acc = 20.0 * (y0 * (d1 - d0) + y1 * (d2 - d1) + y2 * (d3 - d2) + y3 * (d4 - d3))
             / (duration * duration);
}

