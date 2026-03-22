#pragma once

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <string>

class SignalLogger {
public:
    SignalLogger(const std::filesystem::path& output_dir,
                 std::initializer_list<std::string> columns);

    void writeRow(std::initializer_list<double> values);
    const std::filesystem::path& path() const;

private:
    static std::filesystem::path resolveDataDirectory(const std::filesystem::path& output_dir);
    static std::string makeTimestampedFilename();

    std::size_t column_count_;
    std::ofstream file_;
    std::filesystem::path file_path_;
};
