#pragma once
#include "obstacle_tracker.hpp"
#include <metavision/sdk/base/utils/timestamp.h>
#include <array>
#include <deque>
#include <string>

namespace oa {

struct TtcResult {
    // TTC per sektor [lewy, centralny, prawy] w sekundach.
    // < 0 → brak danych; małe dodatnie → bliska kolizja
    std::array<float, 3> ttc = {-1.f, -1.f, -1.f};
    std::string          alert;
    bool                 danger = false;
};

/// Estymator TTC oparty na wzroście NETTO powierzchni bbox w oknie czasowym.
///
/// Realna przeszkoda zbliżająca się powiększa bbox trwale (wzrost netto > 0
/// w całym oknie). Obiekty okresowe „w miejscu” (np. obracający się wiatrak)
/// oscylują wokół linii bazowej — w oknie wzrost netto ≈ 0, więc nie wyzwalają
/// alarmu. Metoda jest odporna na pojedyncze zaszumione klatki (nie wymaga
/// ścisłej serii wzrostów pod rząd).
class TtcEstimator {
public:
    static constexpr float    TTC_DANGER_THRESHOLD_S = 0.45f;
    static constexpr float    MIN_GROWTH_RATE        = 1500.f; // px²/s (netto w oknie)
    static constexpr float    TTC_SMOOTH_ALPHA       = 0.4f;
    static constexpr float    TTC_DECAY_FACTOR       = 0.6f;

    /// Okno analizy wzrostu netto (µs).
    static constexpr int64_t  GROWTH_WINDOW_US       = 60000; // 60 ms

    /// Minimalny względny przyrost pola w oknie (np. 0.25 = +25%),
    /// aby uznać że obiekt realnie się powiększa, a nie tylko drga.
    static constexpr float    MIN_RELATIVE_GROWTH    = 0.25f;

    explicit TtcEstimator() = default;

    TtcResult estimate(const TrackResult& current, Metavision::timestamp ts_us);

private:
    static int bbox_area(const TrackResult& t) noexcept;

    struct AreaSample {
        Metavision::timestamp ts = 0;
        int                   area = 0;
    };

    std::array<std::deque<AreaSample>, 3> history_;
    std::array<float, 3>                  prev_ttc_ = {-1.f, -1.f, -1.f};
    Metavision::timestamp                 prev_timestamp_us_ = -1;
};

} // namespace oa
