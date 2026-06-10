#include <atomic>
#include <csignal>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/base/events/event_cd.h>
// Uwaga: I_EventsStream (HAL) NIE posiada start_recording/stop_recording.
// Do nagrywania RAW używamy camera.start_recording() z SDK Stream API.

#include "bias_configurator.hpp"
#include "neighborhood_filter.hpp"
#include "obstacle_tracker.hpp"
#include "ttc_estimator.hpp"

// ─────────────────────────────────────────────────────────────────────────────
//  Bezpieczna flaga przerwania (Ctrl+C / SIGTERM).
//
//  std::atomic<bool> + memory_order_relaxed:
//    - Bezpieczne do odczytu/zapisu z handlera sygnału (async-signal-safe).
//    - Nie wymaga full memory fence — wystarczy widoczność zmiany.
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) noexcept {
    g_running.store(false, std::memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Konfiguracja
// ─────────────────────────────────────────────────────────────────────────────
static constexpr const char* OUTPUT_RAW_FILE      = "output/wykryty_ruch.raw";
static constexpr const char* OUTPUT_FILTERED_FILE = "output/wykryty_ruch_filtered.raw";

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Optical Avoidance System — RPi 5 / GenX320 ===\n";

    // ── 1. Otwórz kamerę ─────────────────────────────────────────────────────
    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_first_available();
    } catch (const std::exception& e) {
        std::cerr << "[BŁĄD] Nie można otworzyć kamery: " << e.what() << "\n";
        return 1;
    }
    std::cout << "[OK] Kamera GenX320 zainicjalizowana.\n";

    // ── 2. Sprzętowa konfiguracja biasów ─────────────────────────────────────
    try {
        oa::BiasConfigurator::apply(camera);
        std::cout << "[OK] Biasy diff_on/off = 45 zaaplikowane.\n";
    } catch (const std::exception& e) {
        std::cerr << "[OSTRZEŻENIE] Biasy: " << e.what() << "\n";
    }

    // ── 3. Natywne nagrywanie surowego strumienia EVT3 ───────────────────────
    //
    // SDK Stream API: camera.start_recording() / camera.stop_recording().
    // Musi być wywołane PO camera.start() (patrz krok 7).
    // Flaga steruje czy nagrywanie w ogóle aktywować.
    //
    // Co trafia do OUTPUT_RAW_FILE:
    //   Surowe dane EVT3 z sensora, PRZED callbackiem i PRZED filtracją.
    //   Zawiera pełny szum termiczny (~60k evt/40ms bez biasów, ~400 po).
    //   Plik jest w pełni kompatybilny z metavision_player, metavision_viewer
    //   i całym ekosystemem Metavision SDK.
    //
    // Co trafia do OUTPUT_FILTERED_FILE (krok 4):
    //   Wyłącznie zdarzenia strukturalne po filtrze sąsiedztwa — do analizy
    //   ruchu i odtwarzania w bin_into_image.py.
    bool raw_recording_active = false;

    // ── 4. Plik przefiltrowanych zdarzeń (kompatybilny z bin_into_image.py) ──
    //
    // Format: surowe bajty struct EventCD (16 B/event: x[u16], y[u16], p[i16], t[i64]).
    // Odczyt: np.fromfile(path, dtype=EVENT_DTYPE) w bin_into_image.py.
    // Ten plik NIE ma nagłówka Metavision — to celowe (prosto, lekko, kompatybilnie).
    std::ofstream filtered_file(OUTPUT_FILTERED_FILE,
                                std::ios::binary | std::ios::trunc);
    if (!filtered_file.is_open()) {
        std::cerr << "[BŁĄD] Nie można otworzyć pliku przefiltrowanego: "
                  << OUTPUT_FILTERED_FILE << "\n";
        return 1;
    }
    std::cout << "[OK] Zapis przefiltrowanych zdarzeń: " << OUTPUT_FILTERED_FILE << "\n\n";

    // ── 5. Moduły algorytmiczne ───────────────────────────────────────────────
    //
    // KONTRAKT WĄTKOWY: poniższe obiekty są WYŁĄCZNIE własnością wątku
    // callbacku SDK (wątek zbierający dane z HAL/V4L2).
    // NIE czytać ani NIE pisać z wątku main() ani żadnego innego
    // bez ochrony przez std::mutex.
    oa::NeighborhoodFilter filter;
    oa::ObstacleTracker    tracker;
    oa::TtcEstimator       ttc_estimator;

    // ── 6. Callback na paczki zdarzeń CD ─────────────────────────────────────
    camera.cd().add_callback(
        [&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
            if (!g_running.load(std::memory_order_relaxed)) return;

            const size_t raw_count = static_cast<size_t>(end - begin);
            if (raw_count == 0) return;

            // ── thread_local bufor przefiltrowanych zdarzeń ──────────────────
            //
            // DLACZEGO thread_local, nie zmienna w main():
            //   Callback wykonuje się w wątku SDK — nie w main().
            //   Zmienna zdefiniowana w main() i przechwycona przez [&] byłaby
            //   mutowana z innego wątku bez synchronizacji → data race → UB.
            //
            // thread_local gwarantuje:
            //   - Obiekt żyje przez cały czas działania wątku SDK (bez re-allokacji).
            //   - Jest wyłączną własnością tego wątku (zero ryzyka wyścigu).
            //   - clear() re-używa zaalokowanej pamięci (capacity rośnie, nie spada).
            thread_local std::vector<Metavision::EventCD> clean_events;
            clean_events.clear();

            // ── Filtracja sąsiedztwa ──────────────────────────────────────────
            const size_t clean_count = filter.filter(begin, raw_count, clean_events);

            // ── Zapis przefiltrowanych zdarzeń do pliku .bin ──────────────────
            //
            // Zapisujemy wyłącznie zdarzenia strukturalne (po filtracji).
            // Plik można otworzyć bezpośrednio w bin_into_image.py.
            if (clean_count > 0) {
                filtered_file.write(
                    reinterpret_cast<const char*>(clean_events.data()),
                    static_cast<std::streamsize>(
                        clean_count * sizeof(Metavision::EventCD))
                );
            }

            // ── Detekcja i tracking ───────────────────────────────────────────
            const oa::TrackResult track = tracker.process(clean_events);

            // ── Szacowanie Time-To-Collision ──────────────────────────────────
            const oa::TtcResult ttc = ttc_estimator.estimate(track);

            // ── Wyświetlanie w konsoli SSH ────────────────────────────────────
            //
            // direction_to_cstr() wywoływana tylko tutaj (display path),
            // nigdy w hot path algorytmicznym.
            if (track.valid) {
                static constexpr const char* SECTOR_NAMES[] =
                    {"LEWY", "CENTRALNY", "PRAWY"};

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
                    << "\r\033[K[STATUS] Scena czysta | Zdarzeń tła: " << clean_count
                    << std::flush;
            }
        }
    );

    // ── 7. Start kamery i pasywna pętla główna ───────────────────────────────
    camera.start();

    // start_recording() musi być wywołane PO camera.start() —
    // wymóg SDK Stream API (strumień musi już płynąć).
    try {
        camera.start_recording(OUTPUT_RAW_FILE);
        raw_recording_active = true;
        std::cout << "[OK] Nagrywanie surowego EVT3: " << OUTPUT_RAW_FILE << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[OSTRZEŻENIE] Nagrywanie RAW wyłączone: " << e.what() << "\n";
    }

    std::cout << "Wciśnij Ctrl+C aby zakończyć i bezpiecznie zapisać...\n";

    while (g_running.load(std::memory_order_relaxed) && camera.is_running()) {
        // Wątek main() śpi — cała praca odbywa się w wątku callbacku SDK.
        // sleep_for(10ms) zamiast busy-wait → ~0% CPU dla main().
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ── 8. Bezpieczne zamknięcie ──────────────────────────────────────────────
    // stop_recording() przed camera.stop() — SDK wymaga tej kolejności.
    if (raw_recording_active) {
        try {
            camera.stop_recording(OUTPUT_RAW_FILE);
            std::cout << "\n\n[OK] Strumień surowy EVT3 zamknięty: " << OUTPUT_RAW_FILE << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[OSTRZEŻENIE] stop_recording: " << e.what() << "\n";
        }
    }
    camera.stop();

    // Flush i zamknięcie pliku przefiltrowanego
    filtered_file.flush();
    filtered_file.close();
    std::cout << "[OK] Plik przefiltrowany zamknięty:   " << OUTPUT_FILTERED_FILE << "\n";
    std::cout << "[OK] System unikania przeszkód zatrzymany.\n";

    return 0;
}
