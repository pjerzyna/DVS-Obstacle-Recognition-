#include "obstacle_tracker.hpp"
#include "sensor_config.hpp"
#include <limits>
#include <cstdlib>

namespace oa {

void ObstacleTracker::set_thresholds(int detection_thresh, int sector_min_events) {
    detection_thresh_  = detection_thresh;
    sector_min_events_ = sector_min_events;
}

int ObstacleTracker::sector_for_x(int x) noexcept {
    if (x <= SECTOR_LEFT_END)       return 0;
    if (x >= SECTOR_RIGHT_START)    return 2;
    return 1;
}

int ObstacleTracker::bbox_area(int w, int h) noexcept {
    return std::max(1, w) * std::max(1, h);
}

TrackResult ObstacleTracker::process(const std::vector<Metavision::EventCD>& events,
                                     Metavision::timestamp slice_end_ts) {
    TrackResult result;
    result.total_events    = events.size();
    result.slice_end_ts    = slice_end_ts;
    result.active_points   = 0;

    const size_t relative_detection_thresh = std::max(
        static_cast<size_t>(config::MIN_DETECTION_THRESH),
        events.size() * static_cast<size_t>(config::SLICE_EVENT_RATIO_PCT) / 100);
    const size_t effective_detection_thresh = std::min(
        static_cast<size_t>(detection_thresh_), relative_detection_thresh);

    if (events.size() < effective_detection_thresh) {
        prev_cx_ = -1;
        prev_cy_ = -1;
        prev_dominant_sector_ = -1;
        return result;
    }

    struct SectorStats {
        size_t  count = 0;
        int     x_min = std::numeric_limits<int>::max();
        int     y_min = std::numeric_limits<int>::max();
        int     x_max = std::numeric_limits<int>::min();
        int     y_max = std::numeric_limits<int>::min();
        int64_t sum_x = 0;
        int64_t sum_y = 0;
    };

    SectorStats sectors[3];

    const size_t n = events.size();
    const Metavision::EventCD* data = events.data();

    for (size_t i = 0; i < n; ++i) {
        const int x = static_cast<int>(data[i].x);
        const int y = static_cast<int>(data[i].y);
        SectorStats& s = sectors[sector_for_x(x)];

        ++s.count;
        s.x_min = (x < s.x_min) ? x : s.x_min;
        s.x_max = (x > s.x_max) ? x : s.x_max;
        s.y_min = (y < s.y_min) ? y : s.y_min;
        s.y_max = (y > s.y_max) ? y : s.y_max;
        s.sum_x += x;
        s.sum_y += y;
    }

    int best_sector = 1;
    size_t best_count = sectors[1].count;
    for (int s = 0; s < 3; ++s) {
        if (sectors[s].count > best_count) {
            best_count = sectors[s].count;
            best_sector = s;
        } else if (sectors[s].count == best_count && best_count > 0) {
            if (s == 1) {
                best_sector = 1;
            } else if (s == prev_dominant_sector_) {
                best_sector = s;
            }
        }
    }

    result.active_points = best_count;

    const size_t relative_sector_min = std::max(
        static_cast<size_t>(config::MIN_SECTOR_MIN_EVENTS),
        events.size() * static_cast<size_t>(config::SECTOR_EVENT_RATIO_PCT) / 100);
    const size_t effective_sector_min = std::min(
        static_cast<size_t>(sector_min_events_), relative_sector_min);

    if (best_count < effective_sector_min) {
        prev_cx_ = -1;
        prev_cy_ = -1;
        prev_dominant_sector_ = -1;
        return result;
    }

    const SectorStats& active = sectors[best_sector];
    result.dominant_sector = best_sector;
    result.cx      = static_cast<int>(active.sum_x / static_cast<int64_t>(active.count));
    result.cy      = static_cast<int>(active.sum_y / static_cast<int64_t>(active.count));
    result.bbox_x1 = active.x_min;
    result.bbox_y1 = active.y_min;
    result.bbox_x2 = active.x_max;
    result.bbox_y2 = active.y_max;
    result.bbox_w  = active.x_max - active.x_min;
    result.bbox_h  = active.y_max - active.y_min;
    result.valid   = true;

    if (prev_dominant_sector_ >= 0 && prev_dominant_sector_ != best_sector && prev_cx_ >= 0) {
        const int jump = std::abs(result.cx - prev_cx_) + std::abs(result.cy - prev_cy_);
        if (jump > SECTOR_JUMP_RESET) {
            prev_cx_ = -1;
            prev_cy_ = -1;
        }
    }

    result.direction = Direction::STATIONARY;
    if (prev_cx_ >= 0) {
        const int dx = result.cx - prev_cx_;
        const int dy = result.cy - prev_cy_;
        result.dx = dx;
        result.dy = dy;

        const int adx = std::abs(dx);
        const int ady = std::abs(dy);

        if (adx > ady && adx > MOVEMENT_THRESH) {
            result.direction = (dx > 0) ? Direction::RIGHT : Direction::LEFT;
        } else if (ady >= adx && ady > MOVEMENT_THRESH) {
            result.direction = (dy > 0) ? Direction::DOWN : Direction::UP;
        }
    }

    prev_cx_ = result.cx;
    prev_cy_ = result.cy;
    prev_dominant_sector_ = best_sector;

    return result;
}

} // namespace oa
