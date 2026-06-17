#include "detection_pipeline.hpp"

#include "sensor_config.hpp"

namespace oa {

DetectionPipeline::DetectionPipeline(int sensor_w, int sensor_h)
    : filter_(sensor_w, sensor_h),
      slicer_(config::SLICE_DURATION_US) {}

FrameOutput DetectionPipeline::process_slice(TimeSlice slice) {
    thresholds_.observe_slice(slice.events.size());

    tracker_.set_thresholds(thresholds_.detection_thresh(),
                            thresholds_.sector_min_events());

    FrameOutput out;
    out.events  = std::move(slice.events);
    out.end_ts  = slice.end_ts;
    out.track   = tracker_.process(out.events, slice.end_ts);
    out.ttc     = ttc_estimator_.estimate(out.track, slice.end_ts);
    return out;
}

std::vector<FrameOutput> DetectionPipeline::process(
    const Metavision::EventCD* begin,
    const Metavision::EventCD* end)
{
    std::vector<FrameOutput> outputs;
    const size_t raw_count = static_cast<size_t>(end - begin);
    if (raw_count == 0) {
        return outputs;
    }

    last_filtered_.clear();
    filter_.filter(begin, raw_count, last_filtered_);

    if (!last_filtered_.empty()) {
        slicer_.push(last_filtered_.data(), last_filtered_.size());
    }

    TimeSlice slice;
    while (slicer_.pop_ready(slice)) {
        outputs.push_back(process_slice(std::move(slice)));
    }

    return outputs;
}

std::vector<FrameOutput> DetectionPipeline::flush() {
    std::vector<FrameOutput> outputs;
    TimeSlice slice;
    if (slicer_.flush(slice)) {
        outputs.push_back(process_slice(std::move(slice)));
    }
    return outputs;
}

} // namespace oa
