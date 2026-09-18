#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <Eigen/Dense>

class ParamterIdentify {
public:
    explicit ParamterIdentify(const std::string& urdf_path);
    ~ParamterIdentify();

    // 参数数量，自由度；预先resize内存，防止运行时分配内存
    void application_memory(int param_num, int dof);

    // 返回剩余的可记录元素数量
    size_t record_value(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel, const Eigen::VectorXd& torque);

    bool reset_recording();
    bool save_csv(std::string_view file_name);
    bool save_is_finished() const;
    bool save_succeeded() const;
    std::size_t record_count() const;

private:
    using Clock = std::chrono::steady_clock;
    static constexpr std::size_t kMaxCsvPathLength = 512;

    void save_worker_func();
    bool write_csv_file(std::string_view file_name, std::size_t sample_count) const;

    std::size_t record_index{0};
    Eigen::Index dof_{0};

    std::atomic_bool save_requested_{false};
    std::atomic_bool save_finished_{true};
    std::atomic_bool save_succeeded_{false};
    std::atomic_bool exit_save_thread_{false};
    std::atomic_size_t save_record_count_{0};
    std::atomic_size_t save_file_name_size_{0};
    std::array<char, kMaxCsvPathLength + 1> save_file_name_{};
    std::thread save_thread_;

    std::vector<Clock::time_point> time_point;
    std::vector<Eigen::VectorXd> pos;
    std::vector<Eigen::VectorXd> vel;
    std::vector<Eigen::VectorXd> torque;
};
