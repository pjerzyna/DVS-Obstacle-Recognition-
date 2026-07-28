# Performance and Complexity - optical_avoidance

This document describes the performance metrics built into the detection
pipeline as well as an analysis of computational and parametric complexity.
Instrumentation is optional and zero-overhead when disabled (gated by the
`PerfMetrics*` pointer in `DetectionPipeline`).

Pipeline (shared by `main` live and `replay_viewer`):

```
NeighborhoodFilter ──(per SDK batch)──▶ FrameSlicer(10 ms)
        │                                      │
        ▼                                      ▼
   spatio-temporal filter             ObstacleTracker ──▶ TtcEstimator   (per slice)
```

---

## 1. Performance metrics

### 1.1 How to enable

All three binaries accept the same flags:

```bash
# Headless benchmark (fastest, for reproducible measurements; no GUI)
./build/benchmark /home/sterydy/output/wykryty_ruch.raw \
    --metrics output/metrics_benchmark.csv \
    --metrics-json output/metrics_benchmark_summary.json

# Replay with preview + metrics
./build/replay_viewer /home/sterydy/output/wykryty_ruch.raw --metrics

# Live mode (camera) + metrics
./build/optical_avoidance --metrics
```

- When the path is omitted, files go to the `output/` directory.
- By default, `benchmark` replays the file "as fast as the CPU allows"
  (`FileConfigHints::real_time_playback(false)`); the `--realtime` flag restores
  the recording's original pace.

### 1.2 Metric definitions

The instrumentation (`include/perf_metrics.hpp`) measures `std::chrono::steady_clock`
around each stage and records two levels of data:

**Per SDK batch** (`*_batches.csv`): `raw_events`, `filtered_events`, `filter_us`.

**Per slice (10 ms)** (main CSV): `slice_events`, `tracker_us`, `ttc_us`, `slice_total_us`, `filter_buffer`, `temporal_buffer_`, `ttc_history`, `valid`, `danger`, `sector`, `ttc_value`, `expanding`, `growth_rate`, `relative_growth`.

slice_total_us=tracker+ttc

| Metric | Definition |
|---|---|
| **Throughput** | `total_raw_events` / sum_of_stage_times [Mev/s] |
| **Real-time factor** | event_time_covered / processing_time (×) |
| **Per-stage latency** | mean / median / p95 / p99 / max [µs] for filter, tracker, ttc, tracker+ttc |
| **Budget overrun** | % of slices in which tracker+ttc > 10 ms |
| **Peak buffer** | peak size of `temporal_buffer_` (filter) and `history_` (TTC) |
| **Filter effectiveness** | `1 − filtered/raw` [%] (rejected events) |
| **Detections** | count and % of slices with `valid`, number of episodes and their avg. duration |
| **Collisions (danger)** | count and % of slices with `danger`, number of episodes |
| **Detection latency** | time to the first `valid` slice in the recording [s] |
| **Jitter** | number of `valid` flag toggles per second (stability) |

Statistics are also saved in aggregate form in `*_summary.json`.

### 1.3 Empirical results

Hardware: Raspberry Pi 5 (Cortex-A76), build `Release -O3 -mcpu=cortex-a76`.
Recording: `wykryty_ruch.raw` (320×320, 7.14 million events, ~26.6 s of events).

```
SDK batches:          26,386
Slices (10 ms):       2,663
Raw events:           7,143,802
Events after filter:  5,171,847   (27.6% rejected)
Throughput:           20.4 Mev/s
Real-time factor:     76×           (350 ms CPU for 26.6 s of events)
Budget overrun 10ms:  0.00%
Peak filter buffer:   9,830 events
Peak TTC history:     7 samples
```

| Stage | mean | median | p95 | p99 | max | [µs] |
|---|---|---|---|---|---|---|
| filter (batch) | 11.7 | 9.3 | 21.5 | 34.3 | 1618 | |
| tracker (slice) | 14.2 | 11.0 | 40.0 | 97.1 | 838 | |
| ttc (slice) | 0.9 | 0.1 | 3.0 | 20.1 | 91.0 | |
| tracker+ttc | 15.1 | 11.3 | 43.0 | 104.5 | 839 | |

Quality (on this recording): 2,496 detections (93.7% of slices), 375 danger
slices (14.1%), 19 detection episodes (avg. 1.3 s), 160 collision episodes,
jitter 1.4/s.

Plots: `output/plots/latency_vs_events.png`, `latency_hist.png`,
`detections_timeline.png`, `ttc_threshold_sweep.png`.

---

## 2. Computational complexity

Notation: `N` — events in a slice, `B` — events in an SDK batch,
`W×H` — sensor dimensions (320×320 = 102,400), `Buf` — size of the filter's
temporal buffer, `Hwin = GROWTH_WINDOW_US / SLICE_DURATION_US` — number of
samples in the TTC window (≈ 6).

| Stage | Dominant operation | Complexity | Notes |
|---|---|---|---|
| `NeighborhoodFilter::filter` | `build_map()` → `std::fill` over the whole map | **O(B + W·H + Buf)** | `W·H` dominates with small batches (avg. 270 events); see below |
| → `purge_old` | `pop_front` from deque | O(removed) | |
| → neighborhood loop | 4 lookups/event | O(B) | O(1) per event |
| `FrameSlicer::push` | `push_back` + emission | O(B) | |
| `FrameSlicer::pop_ready` | `erase(begin())` on a `vector` | **O(R)** per pop | R = ready slices; usually 1, but O(R²) when catching up on a backlog |
| `ObstacleTracker::process` | single pass over events | **O(N)** | bbox/centroid/sectors in one pass |
| `TtcEstimator::estimate` | sliding window on a deque | **O(Hwin) ≈ O(1)** | Hwin bounded (peak 7) |

### 2.1 Empirical confirmation

- **Tracker = O(N), linear.** Linear fit: ≈ 9.06 ns/event
  (`latency_vs_events.png`). The constant term is close to 0, i.e. no overhead
  beyond the pass over events.
- **TTC ≈ O(1).** Median 0.1 µs, `peak_ttc_history = 7` — independent of `N`.
- **Filter dominated by `W·H`, not by events.** The average batch has ~270
  events, while `filter_us` has a median of 9.3 µs — which corresponds to a
  `std::fill` of a ~100 kB map (memset), not the number of events. This is the
  main fixed-cost bottleneck:
  `26,386 batches × 102,400 cells ≈ 2.7 billion` map-zeroing operations.

### 2.2 Identified bottlenecks and optimization directions

1. **`build_map()` clears the entire map every batch (O(W·H)).** Because SDK
   batches are small, zeroing the map dominates the filter's cost. Possible
   improvements (without changing results): a "marker" map with a timestamp
   instead of a counter (no clearing), or clearing only the cells touched in
   the previous window.
2. **`FrameSlicer::pop_ready` uses `erase(begin())` on a `std::vector` (O(R)).**
   Under normal operation R≈1, but when catching up on a backlog it becomes
   O(R²). A `std::deque` or a read index instead of `erase` would be cheaper.
3. **The latency tail (max filter 1.6 ms, tracker 0.84 ms)** comes from rare
   large batches/slices (up to 26 thousand events). p99 is still < 105 µs ≪ 10 ms —
   no real-time budget overruns.

The current headroom is large (real-time factor 76×, 0% budget overruns), so
the above are "just-in-case" optimizations / for larger scenes, not a necessity.

---

## 3. Parametric complexity

### 3.1 Parameter inventory

All parameters below can be overridden at compile time (`OA_*` macros)
without changing the default values — which enables sweeping
(`tools/param_sweep.py sweep`). The default behavior remains unchanged.

| Parameter | File | Default | Sensible range | Meaning / dependencies |
|---|---|---|---|---|
| `OA_FILTER_MIN_NEIGHBORS` | `neighborhood_filter.hpp` | 1 | 1–4 | ↑ = stronger noise rejection, fewer detections, faster tracker |
| `OA_TEMPORAL_WINDOW_US` | `sensor_config.hpp` | 3000 | 1000–8000 | temporal neighborhood window; ↑ = larger `Buf`, more context |
| `OA_SLICE_DURATION_US` | `sensor_config.hpp` | 10000 | 5000–20000 | tracker/TTC window = real-time budget; affects `Hwin` |
| `DETECTION_THRESH` / `SECTOR_MIN_EVENTS` | `obstacle_tracker.hpp` | 45 / 15 | 20–200 / 10–80 | thresholds for treating a slice as a detection; tuned adaptively |
| `MIN_DETECTION_THRESH` / `MIN_SECTOR_MIN_EVENTS` | `sensor_config.hpp` | 35 / 10 | — | lower bound for the adaptive thresholds |
| `SLICE_EVENT_RATIO_PCT` / `SECTOR_EVENT_RATIO_PCT` | `sensor_config.hpp` | 12 / 8 | 5–25 | relative thresholds (% of events) — relax the threshold with weak signal |
| `OA_TTC_GROWTH_WINDOW_US` | `ttc_estimator.hpp` | 60000 | 30000–120000 | net bbox growth window; defines `Hwin` |
| `OA_TTC_MIN_GROWTH_RATE` | `ttc_estimator.hpp` | 1500 px²/s | 500–4000 | min. area growth rate → rejects a fan etc. |
| `OA_TTC_MIN_RELATIVE_GROWTH` | `ttc_estimator.hpp` | 0.25 | 0.1–0.5 | min. relative growth in the window → rejects oscillations |
| `OA_TTC_DANGER_THRESHOLD_S` | `ttc_estimator.hpp` | 0.45 s | 0.2–1.0 | collision alarm threshold; ↓ = fewer false alarms, warns later |

Key dependencies:
- `OA_TTC_GROWTH_WINDOW_US / OA_SLICE_DURATION_US = Hwin` — number of samples in
  the TTC window. Shortens/lengthens the estimator's "memory".
- `MIN_GROWTH_RATE` + `MIN_RELATIVE_GROWTH` together decide on rejecting
  periodic objects (a fan): both must be satisfied simultaneously for
  `expanding=true`.
- `MIN_NEIGHBORS` ↔ `TEMPORAL_WINDOW_US`: a larger window increases the number
  of neighbors, so the same `MIN_NEIGHBORS` becomes more lenient.

### 3.2 Parameter sweeping

**Offline (no recompilation)** — the TTC threshold re-thresholded from the
recorded `ttc_value`/`expanding`:

```bash
python tools/param_sweep.py ttc-sweep output/metrics_benchmark.csv \
       [--labels tools/labels.csv]
```

With an optional labels file (`start_us,end_us` of real collisions) it
generates PR and ROC curves over `TTC_DANGER_THRESHOLD_S`. Without labels — a
plot of collision count vs. threshold.

**With recompilation** — any macro parameter:

```bash
python tools/param_sweep.py sweep --param OA_FILTER_MIN_NEIGHBORS \
       --values 1,2,3 --raw /home/sterydy/output/wykryty_ruch.raw
```

Example result (`MIN_NEIGHBORS`, same recording):

| MIN_NEIGHBORS | filter rejection | tracker mean | throughput | detections | collisions |
|---|---|---|---|---|---|
| 1 | 27.6% | 18.1 µs | 13.7 Mev/s | 2496 | 375 |
| 2 | 40.3% | 12.0 µs | 16.4 Mev/s | 2402 | 412 |
| 3 | 51.4% | 9.0 µs | 17.7 Mev/s | 2233 | 387 |

The classic trade-off is visible: a more aggressive filter (↑MIN_NEIGHBORS)
speeds up the tracker and raises throughput at the cost of the number of
detections. (Throughput in the sweep is lower than in a single clean run due
to parallel compilation.)

---

## 4. Conclusions

- **Real-time with a large margin:** 76× faster than event time, p99 latency of
  tracker+ttc ≈ 105 µs ≪ the 10 ms budget, 0% overruns.
- **Scaling confirmed:** tracker O(N) (~9 ns/event), TTC O(1),
  filter dominated by the fixed O(W·H) cost of zeroing the map.
- **Main bottleneck:** `build_map()` (zeroing the entire map every batch) —
  candidate no. 1 for optimization with larger sensors / more motion.
- **The parameter tools** make it possible to quantitatively tune sensitivity
  vs. cost and the alarm threshold (PR/ROC), without manually modifying code.
