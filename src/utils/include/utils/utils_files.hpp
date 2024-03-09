#ifndef UTILS_FILES_H
#define UTILS_FILES_H

#include <string>

namespace utils {
    namespace files {
        std::string get_config_file_path(const std::string& pkg_name, const std::string& config_filename = "config.yaml");
    }
}

#endif // UTILS_FILES_H
