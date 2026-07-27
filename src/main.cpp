#include <atomic>
#include <csignal>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>
#include <mutex>
#include <filesystem>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "bias_configurator.hpp"
#include "detection_pipeline.hpp"
#include "metrics_cli.hpp"
#include "output_paths.hpp"
#include "perf_metrics.hpp"
#include "sensor_config.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) noexcept {
    g_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Optical Avoidance System — RPi 5 / GenX320 ===\n";

    const std::filesystem::path output_dir = oa::resolve_output_dir(argc, argv);
    const std::filesystem::path raw_path      = oa::raw_recording_path(output_dir);
    const std::filesystem::path filtered_path = oa::filtered_recording_path(output_dir);

    std::cout << "[INFO] Katalog roboczy (cwd): " << std::filesystem::current_path() << "\n";
    std::cout << "[INFO] Katalog wyjściowy:     " << output_dir << "\n";

    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_first_available();
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD] Nie można otworzyć kamery: " << e.what() << "\n";
        return 1;
    }

    const int sensor_w = static_cast<int>(camera.geometry().get_width());
    const int sensor_h = static_cast<int>(camera.geometry().get_height());
    if (sensor_w != oa::config::DEFAULT_SENSOR_W ||
        sensor_h != oa::config::DEFAULT_SENSOR_H) {
        std::cerr << "[OSTRZEŻENIE] Nieoczekiwana geometria sensora: "
                  << sensor_w << "x" << sensor_h
                  << " (oczekiwano " << oa::config::DEFAULT_SENSOR_W
                  << "x" << oa::config::DEFAULT_SENSOR_H << ")\n";
    }
    std::cout << "[OK] Kamera GenX320 zainicjalizowana (" << sensor_w << "x" << sensor_h << ").\n";

    if (oa::BiasConfigurator::apply(camera)) {
        std::cout << "[OK] Biasy diff_on/off = "
                  << oa::BiasConfigurator::DIFF_ON_VALUE << " zaaplikowane.\n";
    } else {
        std::cerr << "[OSTRZEŻENIE] Biasy nie zostały zaaplikowane — "
                     "oczekiwany wysoki szum termiczny.\n";
    }

    bool raw_recording_active = false;

    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[BŁĄD] Nie można utworzyć katalogu output: "
                  << ec.message() << "\n";
        return 1;
    }

    std::ofstream filtered_file(filtered_path, std::ios::binary | std::ios::trunc);
    if (!filtered_file.is_open()) {
        std::cerr << "[BŁĄD] Nie można otworzyć pliku przefiltrowanego: "
                  << filtered_path << "\n";
        return 1;
    }
    std::cout << "[OK] Zapis przefiltrowanych zdarzeń: " << filtered_path << "\n\n";

    std::mutex file_mutex;
    oa::DetectionPipeline pipeline(sensor_w, sensor_h);

    // ── Opcjonalna instrumentacja wydajności (--metrics / --metrics-json) ────
    const oa::MetricsCliOptions metrics_opt = oa::parse_metrics_cli(argc, argv);
    oa::PerfMetrics metrics(static_cast<double>(oa::config::SLICE_DURATION_US));
    if (metrics_opt.enabled) {
        pipeline.set_metrics(&metrics);
        std::cout << "[OK] Instrumentacja metryk WŁĄCZONA.\n";
    }

    bool prev_danger = false;  // zbocze flagi kolizji między slice'ami

    camera.cd().add_callback(
        [&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
            if (!g_running) return;

            const auto frames = pipeline.process(begin, end);

            const auto& filtered_batch = pipeline.last_filtered_batch();
            if (!filtered_batch.empty()) {
                std::lock_guard<std::mutex> lock(file_mutex);
                filtered_file.write(
                    reinterpret_cast<const char*>(filtered_batch.data()),
                    static_cast<std::streamsize>(
                        filtered_batch.size() * sizeof(Metavision::EventCD))
                );
                if (!filtered_file.good()) {
                    std::cerr << "\n[BŁĄD] Zapis pliku przefiltrowanego nie powiódł się.\n";
                    g_running = false;
                    return;
                }
            }

            for (const auto& frame : frames) {
                const oa::TrackResult& track = frame.track;
                const oa::TtcResult& ttc     = frame.ttc;

                if (track.valid) {
                    static constexpr const char* SECTOR_NAMES[] =
                        {"LEWY", "CENTRALNY", "PRAWY"};

                    // Trwały komunikat na początku epizodu kolizji (zbocze
                    // narastające danger), aby nie został nadpisany przez
                    // jednoliniowy status \r.
                    if (ttc.danger && !prev_danger) {
                        std::cout
                            << "\r\033[K"
                            << "\n[⚠ KOLIZJA] " << ttc.alert
                            << " | Centroid:(" << track.cx << "," << track.cy << ")"
                            << " Sektor:" << SECTOR_NAMES[track.dominant_sector]
                            << "\n";
                    }

                    std::cout
                        << "\r\033[K"
                        << "[DETEKCJA] "
                        << "Centroid:("  << track.cx << "," << track.cy << ") "
                        << "BB:["        << track.bbox_w << "x" << track.bbox_h << "] "
                        << "Kierunek:"   << oa::direction_to_cstr(track.direction) << " "
                        << "Sektor:"     << SECTOR_NAMES[track.dominant_sector] << " "
                        << "Punkty:"     << track.active_points;

                    if (ttc.danger) {
                        std::cout << " ⚠  " << ttc.alert;
                    }
                    std::cout << std::flush;
                } else {
                    std::cout
                        << "\r\033[K[STATUS] Scena czysta | Zdarzeń/slice: "
                        << track.total_events
                        << std::flush;
                }

                prev_danger = ttc.danger;
            }
        }
    );

    camera.start();

    try {
        camera.start_recording(raw_path.string());
        raw_recording_active = true;
        std::cout << "[OK] Nagrywanie surowego EVT3: " << raw_path << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[OSTRZEŻENIE] Nagrywanie RAW wyłączone: " << e.what() << "\n";
    }

    std::cout << "Wciśnij Ctrl+C aby zakończyć i bezpiecznie zapisać...\n";

    while (g_running && camera.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (raw_recording_active) {
        try {
            camera.stop_recording(raw_path.string());
            std::cout << "\n\n[OK] Strumień surowy EVT3 zamknięty: " << raw_path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[OSTRZEŻENIE] stop_recording: " << e.what() << "\n";
        }
    }
    camera.stop();

    for (const auto& frame : pipeline.flush()) {
        if (frame.track.valid) {
            std::cout << "\n[DETEKCJA] Ostatni slice: centroid("
                      << frame.track.cx << "," << frame.track.cy << ")\n";
        }
    }

    {
        std::lock_guard<std::mutex> lock(file_mutex);
        filtered_file.flush();
        filtered_file.close();
    }
    std::cout << "[OK] Plik przefiltrowany zamknięty:   " << filtered_path << "\n";

    if (metrics_opt.enabled) {
        const std::filesystem::path csv_path =
            metrics_opt.csv_path.empty()
                ? (output_dir / "metrics_live.csv")
                : std::filesystem::path(metrics_opt.csv_path);
        const std::filesystem::path json_path =
            metrics_opt.json_path.empty()
                ? (output_dir / "metrics_live_summary.json")
                : std::filesystem::path(metrics_opt.json_path);

        metrics.write_csv(csv_path.string());
        metrics.write_summary_json(json_path.string());
        metrics.print_summary(std::cout);
        std::cout << "[OK] Metryki (CSV):     " << csv_path << "\n";
        std::cout << "[OK] Metryki (JSON):    " << json_path << "\n";
    }

    std::cout << "[OK] System unikania przeszkód zatrzymany.\n";

    return 0;
}
