#include "frame_slicer.hpp"

namespace oa {

FrameSlicer::FrameSlicer(int64_t slice_duration_us)
    : slice_duration_us_(slice_duration_us) {}

void FrameSlicer::emit_slice(TimeSlice& out) {
    out.events.swap(buffer_);
    out.end_ts = next_slice_end_ts_;
    buffer_.clear();
    buffer_.reserve(4096);
}

void FrameSlicer::push(const Metavision::EventCD* begin, size_t count) {
    if (count == 0) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        const Metavision::EventCD& ev = begin[i];

        if (next_slice_end_ts_ < 0) {
            next_slice_end_ts_ = ev.t + slice_duration_us_;
        }

        while (ev.t >= next_slice_end_ts_) {
            TimeSlice slice;
            emit_slice(slice);
            ready_.push_back(std::move(slice));
            next_slice_end_ts_ += slice_duration_us_;
        }

        buffer_.push_back(ev);
    }
}

bool FrameSlicer::pop_ready(TimeSlice& out) {
    if (ready_.empty()) {
        return false;
    }
    out = std::move(ready_.front());
    ready_.erase(ready_.begin());
    return true;
}

bool FrameSlicer::flush(TimeSlice& out) {
    if (buffer_.empty()) {
        return false;
    }
    if (next_slice_end_ts_ < 0) {
        next_slice_end_ts_ = buffer_.back().t;
    }
    emit_slice(out);
    next_slice_end_ts_ = -1;
    return true;
}

} // namespace oa
