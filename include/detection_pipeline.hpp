#pragma once

#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "adaptive_thresholds.hpp"
#include "frame_slicer.hpp"
#include "neighborhood_filter.hpp"
#include "obstacle_tracker.hpp"
#include "perf_metrics.hpp"
#include "ttc_estimator.hpp"

namespace oa {

struct FrameOutput {
    TrackResult                        track;
    TtcResult                          ttc;
    std::vector<Metavision::EventCD>   events;
    Metavision::timestamp              end_ts = 0;
};

/// Wspólna ścieżka: filtr → slicer → tracker → TTC (live i replay).
class DetectionPipeline {
public:
    explicit DetectionPipeline(int sensor_w, int sensor_h);

    std::vector<FrameOutput> process(const Metavision::EventCD* begin,
                                     const Metavision::EventCD* end);

    std::vector<FrameOutput> flush();

    /// Przefiltrowane zdarzenia z ostatniej paczki SDK (do zapisu pliku).
    const std::vector<Metavision::EventCD>& last_filtered_batch() const noexcept {
        return last_filtered_;
    }

    /// Włącza instrumentację. nullptr (domyślnie) = zero narzutu.
    void set_metrics(PerfMetrics* metrics) noexcept { metrics_ = metrics; }

private:
    FrameOutput process_slice(TimeSlice slice);

    NeighborhoodFilter   filter_;
    FrameSlicer          slicer_;
    ObstacleTracker      tracker_;
    TtcEstimator         ttc_estimator_;
    AdaptiveThresholds   thresholds_;

    std::vector<Metavision::EventCD> last_filtered_;
    PerfMetrics*                     metrics_ = nullptr;
};

} // namespace oa
