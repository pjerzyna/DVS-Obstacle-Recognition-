#include "neighborhood_filter.hpp"
#include "sensor_config.hpp"

#include <algorithm>

namespace oa {

NeighborhoodFilter::NeighborhoodFilter(int sensor_w, int sensor_h)
    : sensor_w_(sensor_w),
      sensor_h_(sensor_h),
      event_map_(static_cast<size_t>(sensor_w) * static_cast<size_t>(sensor_h), 0) {}

void NeighborhoodFilter::purge_old(Metavision::timestamp newest_ts) {
    const Metavision::timestamp cutoff = newest_ts - config::TEMPORAL_WINDOW_US;
    while (!temporal_buffer_.empty() && temporal_buffer_.front().t < cutoff) {
        temporal_buffer_.pop_front();
    }
}

void NeighborhoodFilter::build_map() {
    std::fill(event_map_.begin(), event_map_.end(), 0);

    for (const auto& ev : temporal_buffer_) {
        if (ev.x >= static_cast<unsigned>(sensor_w_) ||
            ev.y >= static_cast<unsigned>(sensor_h_)) {
            continue;
        }
        const size_t idx =
            static_cast<size_t>(ev.y) * static_cast<size_t>(sensor_w_) + ev.x;
        if (event_map_[idx] < 255) {
            ++event_map_[idx];
        }
    }
}

size_t NeighborhoodFilter::filter(
    const Metavision::EventCD* __restrict__ events_in,
    size_t                                   count,
    std::vector<Metavision::EventCD>&        events_out)
{
    if (count == 0) {
        return 0;
    }

    for (size_t i = 0; i < count; ++i) {
        temporal_buffer_.push_back(events_in[i]);
    }
    purge_old(events_in[count - 1].t);
    build_map();

    events_out.resize(count);
    size_t out_count = 0;

    for (size_t i = 0; i < count; ++i) {
        const int x = static_cast<int>(events_in[i].x);
        const int y = static_cast<int>(events_in[i].y);

        if (x >= sensor_w_ || y >= sensor_h_) {
            continue;
        }

        if (i + 8 < count) {
            const auto& next_ev = events_in[i + 8];
            if (next_ev.x < static_cast<unsigned>(sensor_w_) &&
                next_ev.y < static_cast<unsigned>(sensor_h_)) {
                const size_t pidx =
                    static_cast<size_t>(next_ev.y) * static_cast<size_t>(sensor_w_) +
                    next_ev.x;
                __builtin_prefetch(&event_map_[pidx], 0, 1);
            }
        }

        const size_t idx =
            static_cast<size_t>(y) * static_cast<size_t>(sensor_w_) + static_cast<size_t>(x);

        const uint8_t n_north = (y > 0) ? event_map_[idx - static_cast<size_t>(sensor_w_)] : 0u;
        const uint8_t n_south = (y < sensor_h_ - 1)
                                    ? event_map_[idx + static_cast<size_t>(sensor_w_)]
                                    : 0u;
        const uint8_t n_west  = (x > 0) ? event_map_[idx - 1] : 0u;
        const uint8_t n_east  = (x < sensor_w_ - 1) ? event_map_[idx + 1] : 0u;

        if (static_cast<int>(n_north + n_south + n_west + n_east) >= MIN_NEIGHBORS) {
            events_out[out_count++] = events_in[i];
        }
    }

    events_out.resize(out_count);
    return out_count;
}

} // namespace oa
