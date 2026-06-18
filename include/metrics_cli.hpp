#pragma once

// Parsowanie flag CLI związanych z metrykami — współdzielone przez main,
// replay_viewer i benchmark. Header-only, bez dodatkowych zależności.

#include <string>
#include <string_view>

namespace oa {

struct MetricsCliOptions {
    bool        enabled   = false;
    std::string csv_path;   // pusty → domyślna nazwa w katalogu output
    std::string json_path;  // pusty → domyślna nazwa w katalogu output
};

inline MetricsCliOptions parse_metrics_cli(int argc, char** argv) {
    MetricsCliOptions opt;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--metrics") {
            opt.enabled = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opt.csv_path = argv[++i];
            }
        } else if (arg == "--metrics-json") {
            opt.enabled = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opt.json_path = argv[++i];
            }
        }
    }
    return opt;
}

} // namespace oa
