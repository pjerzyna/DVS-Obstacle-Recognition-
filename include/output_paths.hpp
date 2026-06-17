#pragma once

#include <filesystem>
#include <string>

namespace oa {

/// Ustala katalog output/ niezależnie od cwd.
/// Priorytet: --output-dir > OA_OUTPUT_DIR > ../output (gdy exe w build/) > ./output
std::filesystem::path resolve_output_dir(int argc, char** argv);

std::filesystem::path raw_recording_path(const std::filesystem::path& output_dir);
std::filesystem::path filtered_recording_path(const std::filesystem::path& output_dir);

} // namespace oa
