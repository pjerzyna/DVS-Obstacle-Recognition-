#pragma once
#include <metavision/sdk/base/events/event_cd.h>
#include <deque>
#include <vector>
#include <cstdint>

namespace oa {

// ─────────────────────────────────────────────────────────────────────────────
//  NeighborhoodFilter — filtr czasowo-przestrzenny (okno TEMPORAL_WINDOW_US).
//
//  Zdarzenie przeżywa, jeśli ≥ MIN_NEIGHBORS sąsiadów (N/S/E/W) było
//  aktywnych w oknie czasowym. Eliminuje hot-pixels rozproszone w czasie.
//
//  KONTRAKT WĄTKOWY: obiekt jest własnością wyłącznie wątku callbacku SDK.
// ─────────────────────────────────────────────────────────────────────────────
class NeighborhoodFilter {
public:
    static constexpr int MIN_NEIGHBORS = 1;

    explicit NeighborhoodFilter(int sensor_w, int sensor_h);

    size_t filter(
        const Metavision::EventCD* __restrict__ events_in,
        size_t                                   count,
        std::vector<Metavision::EventCD>&        events_out
    );

    int sensor_w() const noexcept { return sensor_w_; }
    int sensor_h() const noexcept { return sensor_h_; }

private:
    void purge_old(Metavision::timestamp newest_ts);
    void build_map();

    int sensor_w_;
    int sensor_h_;

    std::deque<Metavision::EventCD> temporal_buffer_;
    std::vector<uint8_t>            event_map_;
};

} // namespace oa
