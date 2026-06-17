#pragma once
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <vector>

namespace oa {

enum class Direction : uint8_t {
    STATIONARY = 0,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

inline const char* direction_to_cstr(Direction d) noexcept {
    switch (d) {
        case Direction::LEFT:       return "W LEWO";
        case Direction::RIGHT:      return "W PRAWO";
        case Direction::UP:         return "W GORĘ";
        case Direction::DOWN:       return "W DÓŁ";
        case Direction::STATIONARY: // fall-through
        default:                    return "STACJONARNY";
    }
}

struct TrackResult {
    int cx      = 0, cy      = 0;
    int bbox_x1 = 0, bbox_y1 = 0;
    int bbox_x2 = 0, bbox_y2 = 0;
    int bbox_w  = 0, bbox_h  = 0;

    int       dx = 0, dy = 0;
    Direction direction = Direction::STATIONARY;

    int    dominant_sector = 1;
    size_t active_points   = 0;  // zdarzenia w sektorze dominującym
    size_t total_events    = 0;  // wszystkie zdarzenia w slice
    bool   valid           = false;

    Metavision::timestamp slice_end_ts = 0;
};

/// Wyznacza BBox i centroid w sektorze z największą aktywnością.
class ObstacleTracker {
public:
    static constexpr int DEFAULT_DETECTION_THRESH  = 45;
    static constexpr int DEFAULT_SECTOR_MIN_EVENTS = 15;
    static constexpr int MOVEMENT_THRESH           = 3;
    static constexpr int SECTOR_JUMP_RESET         = 25;

    static constexpr int SECTOR_LEFT_END    = 106;
    static constexpr int SECTOR_RIGHT_START = 214;

    explicit ObstacleTracker() = default;

    void set_thresholds(int detection_thresh, int sector_min_events);

    TrackResult process(const std::vector<Metavision::EventCD>& events,
                        Metavision::timestamp slice_end_ts);

private:
    static int sector_for_x(int x) noexcept;
    static int bbox_area(int w, int h) noexcept;

    int detection_thresh_  = DEFAULT_DETECTION_THRESH;
    int sector_min_events_ = DEFAULT_SECTOR_MIN_EVENTS;

    int prev_cx_ = -1, prev_cy_ = -1;
    int prev_dominant_sector_ = -1;
};

} // namespace oa
