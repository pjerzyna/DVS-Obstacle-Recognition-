#pragma once
#include "obstacle_tracker.hpp"
#include <array>
#include <string>

namespace oa {

/// Wynik TTC dla jednej paczki.
struct TtcResult {
    // TTC per sektor [lewy, centralny, prawy]. Wartość w liczbie klatek.
    // < 0 → brak danych, duże > 0 → odległa przeszkoda, ~1..3 → KOLIZJA
    std::array<float, 3> ttc = {-1.f, -1.f, -1.f};
    std::string          alert;   // "ZAGROŻENIE LEWO" / "KOLIZJA CENTRALNIE" / itp.
    bool                 danger = false;
};

/// Lekki estymaator TTC oparty na dynamice Bounding Boxa.
///
/// TTC = rozmiar_BB / prędkość_wzrostu_BB
/// Prędkość wyznaczana jako delta rozmiaru między bieżącą a poprzednią klatką.
/// Nie wymaga kalibracji metrycznej — działa w pikselach.
class TtcEstimator {
public:
    static constexpr float TTC_DANGER_THRESHOLD = 3.0f;  // klatki → ~120 ms przy 40 ms oknie

    explicit TtcEstimator() = default;

    TtcResult estimate(const TrackResult& current);

private:
    int   prev_bbox_w_ = 0, prev_bbox_h_ = 0;
    int   prev_sector_ = 1;

    // Historia rozmiarów BB per sektor (3 sektory)
    std::array<int, 3> prev_sector_size_ = {0, 0, 0};
};

} // namespace oa