#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "detection_pipeline.hpp"
#include "metrics_cli.hpp"
#include "output_paths.hpp"
#include "perf_metrics.hpp"
#include "sensor_config.hpp"
#include <filesystem>

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

static cv::Mat events_to_bgr(const std::vector<Metavision::EventCD>& events,
                             int width, int height) {
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& ev : events) {
        if (ev.x < static_cast<unsigned>(width) && ev.y < static_cast<unsigned>(height)) {
            frame.at<cv::Vec3b>(ev.y, ev.x) = cv::Vec3b(0, 255, 0);
        }
    }
    return frame;
}

static void draw_detection(cv::Mat& frame, const oa::TrackResult& track, const oa::TtcResult& ttc) {
    static constexpr const char* SECTOR_NAMES[] = {"LEWY", "CENTRALNY", "PRAWY"};

    cv::line(frame, cv::Point(oa::ObstacleTracker::SECTOR_LEFT_END, 0),
             cv::Point(oa::ObstacleTracker::SECTOR_LEFT_END, frame.rows - 1),
             cv::Scalar(64, 64, 64), 1);
    cv::line(frame, cv::Point(oa::ObstacleTracker::SECTOR_RIGHT_START, 0),
             cv::Point(oa::ObstacleTracker::SECTOR_RIGHT_START, frame.rows - 1),
             cv::Scalar(64, 64, 64), 1);

    if (!track.valid) {
        cv::putText(frame, "Brak detekcji", cv::Point(5, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        return;
    }

    cv::rectangle(
        frame,
        cv::Rect(track.bbox_x1, track.bbox_y1,
                 std::max(1, track.bbox_w), std::max(1, track.bbox_h)),
        cv::Scalar(0, 0, 255),
        2
    );
    cv::circle(frame, cv::Point(track.cx, track.cy), 3, cv::Scalar(255, 0, 0), -1);

    std::string info = "Sektor: " + std::string(SECTOR_NAMES[track.dominant_sector])
                     + " | " + oa::direction_to_cstr(track.direction)
                     + " | pkt: " + std::to_string(track.active_points);
    cv::putText(frame, info, cv::Point(5, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);

    if (ttc.danger) {
        cv::putText(frame, ttc.alert, cv::Point(5, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string input_path = parse_input_path(argc, argv);
    std::cout << "=== Replay Viewer — detekcja na nagraniu ===\n";
    std::cout << "Plik: " << input_path << "\n";

    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_file(input_path);
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD] Nie można otworzyć pliku: " << e.what() << "\n";
        return 1;
    }

    const int width  = static_cast<int>(camera.geometry().get_width());
    const int height = static_cast<int>(camera.geometry().get_height());
    std::cout << "Rozdzielczość: " << width << "x" << height << "\n";

    oa::DetectionPipeline pipeline(width, height);

    // ── Opcjonalna instrumentacja wydajności ─────────────────────────────────
    const oa::MetricsCliOptions metrics_opt = oa::parse_metrics_cli(argc, argv);
    oa::PerfMetrics metrics(static_cast<double>(oa::config::SLICE_DURATION_US));
    if (metrics_opt.enabled) {
        pipeline.set_metrics(&metrics);
        std::cout << "[OK] Instrumentacja metryk WŁĄCZONA.\n";
    }

    std::mutex frame_mutex;
    cv::Mat display_frame;
    bool frame_ready = false;

    auto show_frame = [&](const oa::FrameOutput& frame) {
        cv::Mat vis = events_to_bgr(frame.events, width, height);
        draw_detection(vis, frame.track, frame.ttc);

        std::lock_guard<std::mutex> lock(frame_mutex);
        display_frame = std::move(vis);
        frame_ready = true;
    };

    camera.cd().add_callback(
        [&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
            if (!g_running) {
                return;
            }

            for (const auto& frame : pipeline.process(begin, end)) {
                show_frame(frame);
            }
        }
    );

    cv::namedWindow("replay_viewer", cv::WINDOW_NORMAL);
    cv::resizeWindow("replay_viewer", 640, 640);
    std::cout << "Sterowanie: ESC lub Q = wyjście\n";

    camera.start();

    while (g_running && camera.is_running()) {
        cv::Mat frame_to_show;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (frame_ready) {
                frame_to_show = display_frame.clone();
            }
        }

        if (!frame_to_show.empty()) {
            cv::imshow("replay_viewer", frame_to_show);
        }

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
            g_running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    camera.stop();

    for (const auto& frame : pipeline.flush()) {
        show_frame(frame);
        cv::Mat last;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            last = display_frame.clone();
        }
        if (!last.empty()) {
            cv::imshow("replay_viewer", last);
            cv::waitKey(200);
        }
    }

    cv::destroyAllWindows();

    if (metrics_opt.enabled) {
        const std::filesystem::path output_dir = oa::resolve_output_dir(argc, argv);
        const std::filesystem::path csv_path =
            metrics_opt.csv_path.empty()
                ? (output_dir / "metrics_replay.csv")
                : std::filesystem::path(metrics_opt.csv_path);
        const std::filesystem::path json_path =
            metrics_opt.json_path.empty()
                ? (output_dir / "metrics_replay_summary.json")
                : std::filesystem::path(metrics_opt.json_path);

        metrics.write_csv(csv_path.string());
        metrics.write_summary_json(json_path.string());
        metrics.print_summary(std::cout);
        std::cout << "[OK] Metryki (CSV):  " << csv_path << "\n";
        std::cout << "[OK] Metryki (JSON): " << json_path << "\n";
    }

    std::cout << "[OK] Odtwarzanie zakończone.\n";
    return 0;
}
