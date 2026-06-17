#include "output_paths.hpp"

#include <cstdlib>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

namespace oa {

static fs::path executable_dir() {
#if defined(__linux__)
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        return exe.parent_path();
    }
#endif
    return fs::current_path();
}

static fs::path parse_output_dir_arg(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if ((arg == "--output-dir" || arg == "-o") && i + 1 < argc) {
            return fs::path(argv[++i]);
        }
    }
    return {};
}

fs::path resolve_output_dir(int argc, char** argv) {
    if (fs::path from_arg = parse_output_dir_arg(argc, argv); !from_arg.empty()) {
        return fs::absolute(from_arg);
    }

    if (const char* env = std::getenv("OA_OUTPUT_DIR")) {
        if (env[0] != '\0') {
            return fs::absolute(fs::path(env));
        }
    }

    const fs::path exe_dir = executable_dir();
    if (exe_dir.filename() == "build") {
        return fs::absolute(exe_dir.parent_path() / "output");
    }

    return fs::absolute(fs::current_path() / "output");
}

fs::path raw_recording_path(const fs::path& output_dir) {
    return output_dir / "wykryty_ruch.raw";
}

fs::path filtered_recording_path(const fs::path& output_dir) {
    return output_dir / "wykryty_ruch_filtered.raw";
}

} // namespace oa
