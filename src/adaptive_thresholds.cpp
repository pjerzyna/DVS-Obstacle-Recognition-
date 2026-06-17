#include "adaptive_thresholds.hpp"

namespace oa {

void AdaptiveThresholds::observe_slice(size_t event_count) {
    if (calibrated_) {
        return;
    }

    samples_.push_back(event_count);
    if (static_cast<int>(samples_.size()) < config::CALIBRATION_SLICES) {
        return;
    }

    std::vector<size_t> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    const size_t median = sorted[sorted.size() / 2];

    // Mnożniki celowo łagodne — poprzednie x4 często blokowało słabe detekcje.
    detection_thresh_ = static_cast<int>(std::max<size_t>(
        config::MIN_DETECTION_THRESH, median * 2));
    sector_min_events_ = static_cast<int>(std::max<size_t>(
        config::MIN_SECTOR_MIN_EVENTS, median / 2 + 5));

    calibrated_ = true;
}

} // namespace oa
