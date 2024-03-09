#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <utils/utils_files.hpp>
#include <stdexcept>

namespace utils {
    namespace files {
        std::string get_config_file_path(const std::string& pkg_name, const std::string& config_filename) {
            try {
                // 获取包的共享目录路径
                std::string package_share_directory = ament_index_cpp::get_package_share_directory(pkg_name);

                // 使用 filesystem::path 来构建路径，以确保路径在不同平台上的正确性
                std::filesystem::path config_file_path = std::filesystem::path(package_share_directory) / "config" / config_filename;

                // 检查文件是否存在
                if (std::filesystem::exists(config_file_path)) {
                    return config_file_path.string();
                } else {
                    throw std::runtime_error("Config file not found: " + config_file_path.string());
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("Error in get_config_file_path: " + std::string(e.what()));
            }
        }
    }
}