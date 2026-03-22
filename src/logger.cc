#include "logger.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

SignalLogger::SignalLogger(const std::filesystem::path& output_dir,
                           std::initializer_list<std::string> columns)
    : column_count_(columns.size()) {
    const std::filesystem::path resolved_dir = resolveDataDirectory(output_dir);
    std::filesystem::create_directories(resolved_dir);

    file_path_ = resolved_dir / makeTimestampedFilename();
    file_.open(file_path_);
    if (!file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + file_path_.string());
    }

    file_ << "#";
    for (const auto& column : columns) {
        file_ << " " << column;
    }
    file_ << '\n';
    file_ << std::fixed << std::setprecision(8);
}

void SignalLogger::writeRow(std::initializer_list<double> values) {
    if (values.size() != column_count_) {
        throw std::runtime_error("Logger row width does not match header width");
    }

    bool first = true;
    for (double value : values) {
        if (!first) {
            file_ << ' ';
        }
        file_ << value;
        first = false;
    }
    file_ << '\n';
}

const std::filesystem::path& SignalLogger::path() const {
    return file_path_;
}

std::filesystem::path SignalLogger::resolveDataDirectory(const std::filesystem::path& output_dir) {
    if (output_dir.is_absolute()) {
        return output_dir;
    }

    if (std::filesystem::exists(output_dir) && std::filesystem::is_directory(output_dir)) {
        return output_dir;
    }

    const std::filesystem::path parent_candidate = std::filesystem::path("..") / output_dir;
    if (std::filesystem::exists(parent_candidate) && std::filesystem::is_directory(parent_candidate)) {
        return parent_candidate;
    }

    return output_dir;
}

std::string SignalLogger::makeTimestampedFilename() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &now_c);
#else
    localtime_r(&now_c, &local_time);
#endif

    std::ostringstream name;
    name << "simulation_"
         << std::put_time(&local_time, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis.count()
         << ".txt";
    return name.str();
}
