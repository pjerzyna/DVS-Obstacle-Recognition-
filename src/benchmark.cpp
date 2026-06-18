// ─────────────────────────────────────────────────────────────────────────────
//  benchmark — headless pomiar wydajności pipeline'u na pliku .raw.
//
//  Karmi DetectionPipeline zdarzeniami z nagrania (bez GUI/OpenCV), z wyłączonym
//  odtwarzaniem w czasie rzeczywistym (przetwarza tak szybko jak CPU pozwala),
//  i zapisuje metryki do CSV + JSON. Przeznaczony do powtarzalnych pomiarów i
//  przemiatania parametrów (zob. tools/param_sweep.py).
//
//  Użycie:
//    benchmark <plik.raw> [--metrics out.csv] [--metrics-json out.json]
//                         [--output-dir DIR] [--realtime]
// ─────────────────────────────────────────────────────────────────────────────

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "detection_pipeline.hpp"
#include "metrics_cli.hpp"
#include "output_paths.hpp"
#include "perf_metrics.hpp"
#include "sensor_config.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) noexcept {
    g_running = false;
}

static std::string parse_input_path(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if ((arg == "--input" || arg == "-i") && i + 1 < argc) {
            return argv[++i];
        }
        if (!arg.empty() && arg[0] != '-') {
            return std::string(arg);
        }
    }
    return oa::raw_recording_path(oa::resolve_output_dir(argc, argv)).string();
}

static bool has_flag(int argc, char** argv, std::string_view flag) {
    for (int i = 1; i < argc; ++i) {
        if (flag == argv[i]) return true;
    }
    return false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string input_path = parse_input_path(argc, argv);
    const bool realtime = has_flag(argc, argv, "--realtime");

    std::cout << "=== Benchmark pipeline (headless) ===\n";
    std::cout << "Plik wejściowy: " << input_path << "\n";
    std::cout << "Tryb czasu:     " << (realtime ? "real-time" : "tak szybko jak CPU") << "\n";

    Metavision::Camera camera;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(realtime);
        camera = Metavision::Camera::from_file(input_path, hints);
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD] Nie można otworzyć pliku: " << e.what() << "\n";
        return 1;
    }

    const int width  = static_cast<int>(camera.geometry().get_width());
    const int height = static_cast<int>(camera.geometry().get_height());
    std::cout << "Rozdzielczość:  " << width << "x" << height << "\n";

    oa::DetectionPipeline pipeline(width, height);
    oa::PerfMetrics metrics(static_cast<double>(oa::config::SLICE_DURATION_US));
    pipeline.set_metrics(&metrics);

    camera.cd().add_callback(
        [&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
            if (!g_running) return;
            // Wynik nie jest używany — liczy się tylko praca pipeline'u.
            volatile size_t produced = pipeline.process(begin, end).size();
            (void)produced;
        }
    );

    camera.start();
    while (g_running && camera.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    camera.stop();

    pipeline.flush();

    const oa::MetricsCliOptions opt = oa::parse_metrics_cli(argc, argv);
    const std::filesystem::path output_dir = oa::resolve_output_dir(argc, argv);
    const std::filesystem::path csv_path =
        opt.csv_path.empty() ? (output_dir / "metrics_benchmark.csv")
                             : std::filesystem::path(opt.csv_path);
    const std::filesystem::path json_path =
        opt.json_path.empty() ? (output_dir / "metrics_benchmark_summary.json")
                              : std::filesystem::path(opt.json_path);

    metrics.write_csv(csv_path.string());
    metrics.write_batch_csv((output_dir /
        (csv_path.stem().string() + "_batches.csv")).string());
    metrics.write_summary_json(json_path.string());
    metrics.print_summary(std::cout);

    std::cout << "[OK] Metryki (CSV):  " << csv_path << "\n";
    std::cout << "[OK] Metryki (JSON): " << json_path << "\n";
    return 0;
}
