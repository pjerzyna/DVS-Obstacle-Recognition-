#pragma once

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <vector>
#include <cstdint>

namespace oa {

struct TimeSlice {
    std::vector<Metavision::EventCD> events;
    Metavision::timestamp              end_ts = 0;
};

/// Dzieli strumień zdarzeń na stałe okna czasowe (np. 10 ms).
class FrameSlicer {
public:
    explicit FrameSlicer(int64_t slice_duration_us);

    void push(const Metavision::EventCD* begin, size_t count);

    bool pop_ready(TimeSlice& out);

    /// Emituje niedomknięty bufor na końcu strumienia.
    bool flush(TimeSlice& out);

    size_t pending_event_count() const noexcept { return buffer_.size(); }

private:
    void emit_slice(TimeSlice& out);

    int64_t                        slice_duration_us_;
    std::vector<Metavision::EventCD> buffer_;
    std::vector<TimeSlice>           ready_;
    Metavision::timestamp            next_slice_end_ts_ = -1;
};

} // namespace oa
