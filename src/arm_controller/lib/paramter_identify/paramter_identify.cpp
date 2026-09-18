#include "paramter_identify.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

ParamterIdentify::ParamterIdentify(const std::string& urdf_path)
    : save_thread_([this]() { save_worker_func(); }) {
    (void)urdf_path;
}

ParamterIdentify::~ParamterIdentify() {
    exit_save_thread_.store(true, std::memory_order_release);
    if (save_thread_.joinable()) {
        save_thread_.join();
    }
}

void ParamterIdentify::application_memory(int param_num, int dof) {
    record_index = 0;
    dof_         = dof > 0 ? dof : 0;

    if (param_num <= 0 || dof_ == 0) {
        time_point.clear();
        pos.clear();
        vel.clear();
        torque.clear();
        return;
    }

    const auto buffer_size = static_cast<std::size_t>(param_num);
    time_point.assign(buffer_size, Clock::time_point{});
    pos.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
    vel.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
    torque.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
}

bool ParamterIdentify::reset_recording() {
    if (save_requested_.load(std::memory_order_acquire)) {
        return false;
    }

    record_index = 0;
    return true;
}

size_t ParamterIdentify::record_value(const Eigen::VectorXd& pos,
                                      const Eigen::VectorXd& vel,
                                      const Eigen::VectorXd& torque) {
    if (save_requested_.load(std::memory_order_acquire) || record_index >= this->pos.size() || dof_ == 0) {
        return 0;
    }

    if (pos.size() != dof_ || vel.size() != dof_ || torque.size() != dof_) {
        return this->pos.size() - record_index;
    }

    time_point[record_index]   = Clock::now();
    this->pos[record_index]    = pos;
    this->vel[record_index]    = vel;
    this->torque[record_index] = torque;
    ++record_index;

    return this->pos.size() - record_index;
}

bool ParamterIdentify::save_csv(std::string_view file_name) {
    if (file_name.empty() || file_name.size() > kMaxCsvPathLength
        || save_requested_.load(std::memory_order_acquire)) {
        return false;
    }

    for (std::size_t i = 0; i < file_name.size(); ++i) {
        save_file_name_[i] = file_name[i];
    }
    save_file_name_[file_name.size()] = '\0';

    save_file_name_size_.store(file_name.size(), std::memory_order_relaxed);
    save_record_count_.store(std::min(record_index, pos.size()), std::memory_order_relaxed);
    save_succeeded_.store(false, std::memory_order_relaxed);
    save_finished_.store(false, std::memory_order_release);
    save_requested_.store(true, std::memory_order_release);
    return true;
}

bool ParamterIdentify::save_is_finished() const {
    return save_finished_.load(std::memory_order_acquire);
}

bool ParamterIdentify::save_succeeded() const {
    return save_succeeded_.load(std::memory_order_acquire);
}

std::size_t ParamterIdentify::record_count() const {
    return record_index;
}

void ParamterIdentify::save_worker_func() {
    while (true) {
        if (save_requested_.load(std::memory_order_acquire)) {
            const std::size_t file_name_size = save_file_name_size_.load(std::memory_order_relaxed);
            const std::size_t sample_count   = save_record_count_.load(std::memory_order_relaxed);
            const std::string file_name(save_file_name_.data(), file_name_size);

            bool save_ok = false;
            try {
                save_ok = write_csv_file(file_name, sample_count);
            } catch (const std::exception& error) {
                std::cerr << "Failed to save parameter identification CSV: " << error.what() << std::endl;
            }

            save_succeeded_.store(save_ok, std::memory_order_release);
            save_requested_.store(false, std::memory_order_release);
            save_finished_.store(true, std::memory_order_release);
            continue;
        }

        if (exit_save_thread_.load(std::memory_order_acquire)) {
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool ParamterIdentify::write_csv_file(std::string_view file_name, std::size_t sample_count) const {
    namespace fs = std::filesystem;

    const fs::path csv_path{std::string(file_name)};
    if (csv_path.has_parent_path()) {
        fs::create_directories(csv_path.parent_path());
    }

    std::ofstream stream(csv_path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
        return false;
    }

    stream << std::setprecision(17);
    stream << "time";
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",pos_" << joint;
    }
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",vel_" << joint;
    }
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",torque_" << joint;
    }
    stream << '\n';

    const std::size_t count = std::min({sample_count, time_point.size(), pos.size(), vel.size(), torque.size()});
    if (count == 0) {
        return static_cast<bool>(stream);
    }

    const Clock::time_point start_time = time_point[0];
    for (std::size_t sample = 0; sample < count; ++sample) {
        const double time_s = std::chrono::duration<double>(time_point[sample] - start_time).count();
        stream << time_s;
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << pos[sample](joint);
        }
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << vel[sample](joint);
        }
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << torque[sample](joint);
        }
        stream << '\n';
    }

    return static_cast<bool>(stream);
}
