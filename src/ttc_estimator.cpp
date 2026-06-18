#include "ttc_estimator.hpp"
#include <sstream>
#include <algorithm>

namespace oa {

int TtcEstimator::bbox_area(const TrackResult& t) noexcept {
    return std::max(1, t.bbox_w) * std::max(1, t.bbox_h);
}

TtcResult TtcEstimator::estimate(const TrackResult& cur, Metavision::timestamp ts_us) {
    TtcResult result;
    result.ttc = prev_ttc_;

    if (!cur.valid) {
        for (int s = 0; s < 3; ++s) {
            history_[s].clear();
            if (result.ttc[s] > 0.f) {
                result.ttc[s] *= TTC_DECAY_FACTOR;
                if (result.ttc[s] < 0.05f) {
                    result.ttc[s] = -1.f;
                }
            }
        }
        prev_ttc_ = result.ttc;
        prev_timestamp_us_ = ts_us;
        return result;
    }

    const int sector   = cur.dominant_sector;
    const int cur_size = bbox_area(cur);

    auto& hist = history_[sector];
    hist.push_back({ts_us, cur_size});

    // Utrzymuj tylko próbki z okna analizy.
    const Metavision::timestamp cutoff = ts_us - GROWTH_WINDOW_US;
    while (hist.size() > 1 && hist.front().ts < cutoff) {
        hist.pop_front();
    }

    float& smoothed = result.ttc[sector];

    const AreaSample& oldest = hist.front();
    const int   base_area  = oldest.area;
    const float dt_s       = static_cast<float>(ts_us - oldest.ts) * 1e-6f;
    const float net_growth = static_cast<float>(cur_size - base_area);

    bool expanding = false;
    if (dt_s > 0.f && base_area > 0) {
        const float growth_rate     = net_growth / dt_s;       // px²/s
        const float relative_growth = net_growth / static_cast<float>(base_area);

        // Diagnostyka/metryki — nie wpływa na decyzję.
        result.growth_rate     = growth_rate;
        result.relative_growth = relative_growth;

        if (growth_rate >= MIN_GROWTH_RATE && relative_growth >= MIN_RELATIVE_GROWTH) {
            expanding = true;
            const float raw_ttc = static_cast<float>(cur_size) / growth_rate;
            if (prev_ttc_[sector] < 0.f) {
                smoothed = raw_ttc;
            } else {
                smoothed = TTC_SMOOTH_ALPHA * raw_ttc
                         + (1.f - TTC_SMOOTH_ALPHA) * prev_ttc_[sector];
            }
        }
    }

    if (!expanding && smoothed > 0.f) {
        // Brak realnej ekspansji (np. wiatrak) → wygaszaj.
        smoothed *= TTC_DECAY_FACTOR;
        if (smoothed < 0.05f) {
            smoothed = -1.f;
        }
    }

    result.expanding = expanding;

    if (smoothed > 0.f && smoothed < TTC_DANGER_THRESHOLD_S && expanding) {
        result.danger = true;
        const char* sector_names[] = {"LEWY", "CENTRALNY", "PRAWY"};
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << "KOLIZJA " << sector_names[sector]
            << " TTC=" << smoothed << " s";
        result.alert = oss.str();
    }

    prev_ttc_ = result.ttc;
    prev_timestamp_us_ = ts_us;

    return result;
}

} // namespace oa
