#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include "obstacle_tracker.hpp"
#include "sensor_config.hpp"

namespace oa {

/// Krótka kalibracja szumu tła na starcie — dostosowuje progi detekcji.
class AdaptiveThresholds {
public:
    void observe_slice(size_t event_count);

    int detection_thresh() const noexcept { return detection_thresh_; }
    int sector_min_events() const noexcept { return sector_min_events_; }
    bool calibrated() const noexcept { return calibrated_; }

private:
    std::vector<size_t> samples_;
    int detection_thresh_  = ObstacleTracker::DEFAULT_DETECTION_THRESH;
    int sector_min_events_ = ObstacleTracker::DEFAULT_SECTOR_MIN_EVENTS;
    bool calibrated_       = false;
};

} // namespace oa
