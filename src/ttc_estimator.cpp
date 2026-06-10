#include "ttc_estimator.hpp"
#include <cmath>
#include <sstream>

namespace oa {

TtcResult TtcEstimator::estimate(const TrackResult& cur) {
    TtcResult result;

    if (!cur.valid) {
        prev_bbox_w_ = 0;
        prev_bbox_h_ = 0;
        prev_sector_size_ = {0, 0, 0};
        return result;
    }

    // ── Rozmiar BB w bieżącej klatce ─────────────────────────────────
    const int cur_size = cur.bbox_w * cur.bbox_h; // powierzchnia [px²]

    // ── Prędkość wzrostu BB (delta powierzchni) ───────────────────────
    const int prev_size = prev_sector_size_[cur.dominant_sector];
    const int delta     = cur_size - prev_size;

    if (delta > 0 && prev_size > 0) {
        // TTC = rozmiar / prędkość_wzrostu [w liczbie klatek]
        result.ttc[cur.dominant_sector] =
            static_cast<float>(cur_size) / static_cast<float>(delta);

        if (result.ttc[cur.dominant_sector] < TTC_DANGER_THRESHOLD) {
            result.danger = true;
            const char* sector_names[] = {"LEWY", "CENTRALNY", "PRAWY"};
            std::ostringstream oss;
            oss << "KOLIZJA " << sector_names[cur.dominant_sector]
                << " TTC=" << result.ttc[cur.dominant_sector] << " kl.";
            result.alert = oss.str();
        }
    }

    // Zapamiętaj rozmiar per sektor
    prev_sector_size_[cur.dominant_sector] = cur_size;

    return result;
}

} // namespace oa