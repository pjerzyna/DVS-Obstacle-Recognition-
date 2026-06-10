#include "obstacle_tracker.hpp"
#include <limits>
#include <cstdlib>  // abs() dla int

namespace oa {

TrackResult ObstacleTracker::process(const std::vector<Metavision::EventCD>& events) {
    TrackResult result;
    result.active_points = events.size();

    if (events.size() < static_cast<size_t>(DETECTION_THRESH)) {
        prev_cx_ = -1;
        prev_cy_ = -1;
        return result; // valid = false
    }

    // ── BBox + Centroid — jedna przepustka, SIMD-friendly ────────────────────
    //
    // #pragma omp simd z reduction pozwala GCC/Clang wektoryzować tę pętlę
    // za pomocą instrukcji ARM NEON: vminq_s32, vmaxq_s32, vaddq_s32.
    // EventCD::x i y są uint16_t (2 B) — konwersja do int jest wymagana,
    // bo min/max numeric_limits<int> nie współpracuje z uint16_t.
    //
    // Cortex-A76 (RPi 5) ma 2x 128-bit NEON ALU → przetworzy 4 int/cykl.
    // Przy typowych ~2000 eventach po filtracji ≈ 500 cykli NEON vs ~2000 skalarnie.
    int x_min =  std::numeric_limits<int>::max();
    int y_min =  std::numeric_limits<int>::max();
    int x_max =  std::numeric_limits<int>::min();
    int y_max =  std::numeric_limits<int>::min();
    int64_t sum_x = 0, sum_y = 0;

    const size_t n = events.size();
    const Metavision::EventCD* data = events.data();

    #pragma omp simd reduction(+:sum_x,sum_y) \
                     reduction(min:x_min,y_min) \
                     reduction(max:x_max,y_max)
    for (size_t i = 0; i < n; ++i) {
        const int x = static_cast<int>(data[i].x);
        const int y = static_cast<int>(data[i].y);
        x_min  = (x < x_min) ? x : x_min;
        x_max  = (x > x_max) ? x : x_max;
        y_min  = (y < y_min) ? y : y_min;
        y_max  = (y > y_max) ? y : y_max;
        sum_x += x;
        sum_y += y;
    }

    result.cx      = static_cast<int>(sum_x / static_cast<int64_t>(n));
    result.cy      = static_cast<int>(sum_y / static_cast<int64_t>(n));
    result.bbox_x1 = x_min;
    result.bbox_y1 = y_min;
    result.bbox_x2 = x_max;
    result.bbox_y2 = y_max;
    result.bbox_w  = x_max - x_min;
    result.bbox_h  = y_max - y_min;
    result.valid   = true;

    // ── Kierunek ruchu ────────────────────────────────────────────────────────
    //
    // Direction to uint8_t enum — zero alokacji na stercie.
    // std::abs() na int (nie float) — unika jednostki FPU w hot path.
    result.direction = Direction::STATIONARY;
    if (prev_cx_ >= 0) {
        const int dx = result.cx - prev_cx_;
        const int dy = result.cy - prev_cy_;
        result.dx = dx;
        result.dy = dy;

        const int adx = std::abs(dx);
        const int ady = std::abs(dy);

        if (adx > ady && adx > MOVEMENT_THRESH) {
            result.direction = (dx > 0) ? Direction::RIGHT : Direction::LEFT;
        } else if (ady >= adx && ady > MOVEMENT_THRESH) {
            result.direction = (dy > 0) ? Direction::DOWN : Direction::UP;
        }
    }
    prev_cx_ = result.cx;
    prev_cy_ = result.cy;

    // ── Dominant sector ───────────────────────────────────────────────────────
    if (result.cx <= SECTOR_LEFT_END)
        result.dominant_sector = 0;
    else if (result.cx >= SECTOR_RIGHT_START)
        result.dominant_sector = 2;
    else
        result.dominant_sector = 1;

    return result;
}

} // namespace oa
