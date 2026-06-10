#include "neighborhood_filter.hpp"
#include <cstring>  // memset

namespace oa {

NeighborhoodFilter::NeighborhoodFilter() {
    std::memset(event_map_, 0, sizeof(event_map_));
}

void NeighborhoodFilter::build_map(
    const Metavision::EventCD* __restrict__ events,
    size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        // Ochrona przed uszkodzonymi pakietami sprzętowymi (Out of Bounds)
        if (events[i].x < SENSOR_W && events[i].y < SENSOR_H) [[likely]] {
            event_map_[events[i].y * SENSOR_W + events[i].x] = 1;
        }
    }
}

void NeighborhoodFilter::selective_clear(
    const Metavision::EventCD* __restrict__ events,
    size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        // Ochrona przed uszkodzonymi pakietami sprzętowymi przy czyszczeniu
        if (events[i].x < SENSOR_W && events[i].y < SENSOR_H) [[likely]] {
            event_map_[events[i].y * SENSOR_W + events[i].x] = 0;
        }
    }
}

size_t NeighborhoodFilter::filter(
    const Metavision::EventCD* __restrict__ events_in,
    size_t                                   count,
    std::vector<Metavision::EventCD>&        events_out)
{
    if (count == 0) return 0;

    build_map(events_in, count);

    events_out.resize(count);
    size_t out_count = 0;

    for (size_t i = 0; i < count; ++i) {
        const int x = static_cast<int>(events_in[i].x);
        const int y = static_cast<int>(events_in[i].y);

        // Jeśli pakiet jest uszkodzony gabarytowo, pomiń go na starcie
        if (x >= SENSOR_W || y >= SENSOR_H) [[unlikely]] {
            continue;
        }

        // Prefetch kolejnych paczek — bezpieczny, sprawdzający granice
        if (i + 8 < count) [[likely]] {
            const auto& next_ev = events_in[i + 8];
            if (next_ev.x < SENSOR_W && next_ev.y < SENSOR_H) [[likely]] {
                __builtin_prefetch(&event_map_[next_ev.y * SENSOR_W + next_ev.x], 0, 1);
            }
        }

        const int idx = y * SENSOR_W + x;

        // Bezpieczne sprawdzanie sąsiadów wewnątrz chronionego indeksu
        const uint8_t n_north = (y > 0)            ? event_map_[idx - SENSOR_W] : 0u;
        const uint8_t n_south = (y < SENSOR_H - 1) ? event_map_[idx + SENSOR_W] : 0u;
        const uint8_t n_west  = (x > 0)            ? event_map_[idx - 1]        : 0u;
        const uint8_t n_east  = (x < SENSOR_W - 1) ? event_map_[idx + 1]        : 0u;

        if (static_cast<int>(n_north + n_south + n_west + n_east) >= MIN_NEIGHBORS) {
            events_out[out_count++] = events_in[i];
        }
    }

    events_out.resize(out_count);

    selective_clear(events_in, count);

    return out_count;
}

} // namespace oa
