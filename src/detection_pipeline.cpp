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

    if (metrics_) {
        const auto t0 = PerfMetrics::Clock::now();
        out.track = tracker_.process(out.events, slice.end_ts);
        const auto t1 = PerfMetrics::Clock::now();
        out.ttc   = ttc_estimator_.estimate(out.track, slice.end_ts);
        const auto t2 = PerfMetrics::Clock::now();

        PerfMetrics::SliceRecord rec;
        rec.end_ts_us      = static_cast<std::int64_t>(slice.end_ts);
        rec.slice_events   = out.events.size();
        rec.tracker_us     = PerfMetrics::to_us(t0, t1);
        rec.ttc_us         = PerfMetrics::to_us(t1, t2);
        rec.slice_total_us = PerfMetrics::to_us(t0, t2);
        rec.filter_buffer  = filter_.temporal_buffer_size();
        rec.ttc_history    = ttc_estimator_.history_size();
        rec.valid          = out.track.valid;
        rec.danger         = out.ttc.danger;
        rec.sector         = out.track.dominant_sector;
        rec.ttc_value      = out.track.valid
                                 ? out.ttc.ttc[out.track.dominant_sector]
                                 : -1.f;
        rec.expanding       = out.ttc.expanding;
        rec.growth_rate     = out.ttc.growth_rate;
        rec.relative_growth = out.ttc.relative_growth;
        metrics_->record_slice(rec);
    } else {
        out.track = tracker_.process(out.events, slice.end_ts);
        out.ttc   = ttc_estimator_.estimate(out.track, slice.end_ts);
    }
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

    if (metrics_) {
        const auto t0 = PerfMetrics::Clock::now();
        filter_.filter(begin, raw_count, last_filtered_);
        const auto t1 = PerfMetrics::Clock::now();
        metrics_->record_batch(raw_count, last_filtered_.size(),
                               PerfMetrics::to_us(t0, t1));
    } else {
        filter_.filter(begin, raw_count, last_filtered_);
    }

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
