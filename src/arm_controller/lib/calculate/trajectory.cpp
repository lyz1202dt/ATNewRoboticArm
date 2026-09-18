#include "trajectory.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace {

// 两点时间间隔过小视为无效段，避免除零。
constexpr double kMinSegmentDuration = 1e-9;

} // namespace

Trajectory::Trajectory(int dof, int max_point_num)
    : copy(dof) {
    points.assign(static_cast<std::size_t>(max_point_num), Point(dof));
}

bool Trajectory::add_point(const Point& point, rclcpp::Duration time_from_start) {
    if (index >= points.size() || points.empty() || point.pos.size() != points.front().pos.size()) {
        return false;
    }

    copy      = point;
    copy.time = time_from_start;

    points[index] = copy;
    index++;
    return true;
}

void Trajectory::start(rclcpp::Time time) {
    start_time_point_ = time;
    started_          = true;
}

void Trajectory::stop() {
    started_ = false;
}

void Trajectory::clear() {
    index    = 0;
    started_ = false;
}

std::size_t Trajectory::size() const {
    return index;
}

std::size_t Trajectory::capacity() const {
    return points.size();
}

bool Trajectory::empty() const {
    return index == 0;
}

const Point& Trajectory::front() const {
    return points.front();
}

const Point& Trajectory::back() const {
    return points[index - 1];
}

void Trajectory::update(rclcpp::Time time, Point& point) {
    if (!started_ || empty()) {
        return;
    }

    const rclcpp::Duration elapsed = time - start_time_point_;

    // 起始点之前与终止点之后保持端点（夹持）。
    if (elapsed <= front().time) {
        point = front();
        return;
    }
    if (elapsed >= back().time) {
        point = back();
        return;
    }

    // 找到包含 elapsed 的段 [segment, segment + 1]。
    size_t segment = 0;
    while (segment + 1 < size() && points[segment + 1].time <= elapsed) {
        ++segment;
    }

    const double start_seconds = points[segment].time.seconds();
    const double end_seconds   = points[segment + 1].time.seconds();
    const double duration      = end_seconds - start_seconds;
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

    // 五次 Bernstein 基函数。
    const double u  = s;
    const double v  = 1.0 - u;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double v2 = v * v;
    const double v3 = v2 * v;
    const double v4 = v3 * v;

    const double w0 = v4 * v;         // (1-u)^5
    const double w1 = 5.0 * u * v4;   // 5u(1-u)^4
    const double w2 = 10.0 * u2 * v3; // 10u^2(1-u)^3
    const double w3 = 10.0 * u3 * v2; // 10u^3(1-u)^2
    const double w4 = 5.0 * u4 * v;   // 5u^4(1-u)
    const double w5 = u4 * u;         // u^5

    // 一阶导（对 s）：B'(s) = 5 · 四次贝塞尔，控制点为 Δi = B_{i+1} - B_i。
    const double z0 = v4;            // (1-u)^4
    const double z1 = 4.0 * u * v3;  // 4u(1-u)^3
    const double z2 = 6.0 * u2 * v2; // 6u^2(1-u)^2
    const double z3 = 4.0 * u3 * v;  // 4u^3(1-u)
    const double z4 = u4;            // u^4

    // 二阶导（对 s）：B''(s) = 20 · 三次贝塞尔，控制点为 Δ_{i+1} - Δ_i。
    const double y0 = v3;           // (1-u)^3
    const double y1 = 3.0 * u * v2; // 3u(1-u)^2
    const double y2 = 3.0 * u2 * v; // 3u^2(1-u)
    const double y3 = u3;           // u^3

    if (out->pos.size() != dimension) {
        out->pos.resize(dimension);
    }
    if (out->vel.size() != dimension) {
        out->vel.resize(dimension);
    }
    if (out->acc.size() != dimension) {
        out->acc.resize(dimension);
    }

    const bool has_a_vel = a.vel.size() == dimension;
    const bool has_b_vel = b.vel.size() == dimension;
    const bool has_a_acc = a.acc.size() == dimension;
    const bool has_b_acc = b.acc.size() == dimension;
    const double t       = duration;

    for (int i = 0; i < dimension; ++i) {
        const double p0 = a.pos(i);
        const double p1 = b.pos(i);
        const double v0 = has_a_vel ? a.vel(i) : 0.0;
        const double v1 = has_b_vel ? b.vel(i) : 0.0;
        const double a0 = has_a_acc ? a.acc(i) : 0.0;
        const double a1 = has_b_acc ? b.acc(i) : 0.0;

        // 五次贝塞尔控制点（等价于五次 Hermite 插值）。
        const double b0 = p0;
        const double b1 = p0 + (t / 5.0) * v0;
        const double b2 = p0 + (2.0 * t / 5.0) * v0 + (t * t / 20.0) * a0;
        const double b3 = p1 - (2.0 * t / 5.0) * v1 + (t * t / 20.0) * a1;
        const double b4 = p1 - (t / 5.0) * v1;
        const double b5 = p1;

        const double d0 = b1 - b0;
        const double d1 = b2 - b1;
        const double d2 = b3 - b2;
        const double d3 = b4 - b3;
        const double d4 = b5 - b4;

        out->pos(i) = w0 * b0 + w1 * b1 + w2 * b2 + w3 * b3 + w4 * b4 + w5 * b5;
        out->vel(i) = 5.0 * (z0 * d0 + z1 * d1 + z2 * d2 + z3 * d3 + z4 * d4) / duration;
        out->acc(i) = 20.0 * (y0 * (d1 - d0) + y1 * (d2 - d1) + y2 * (d3 - d2) + y3 * (d4 - d3)) / (duration * duration);
    }
}
