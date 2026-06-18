#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  PerfMetrics — lekka, nagłówkowa instrumentacja wydajności pipeline'u.
//
//  Projekt:
//   * Zero narzutu gdy nieaktywna — DetectionPipeline trzyma wskaźnik
//     PerfMetrics*, a wszystkie pomiary są bramkowane sprawdzeniem != nullptr.
//   * Brak ciężkich zależności — tylko biblioteka standardowa.
//   * Per-slice CSV (do analizy skalowania i offline'owego przemiatania progu
//     TTC) + zbiorczy JSON/summary (latencje, throughput, jakość detekcji).
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

namespace oa {

class PerfMetrics {
public:
    using Clock = std::chrono::steady_clock;

    static double to_us(Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::micro>(b - a).count();
    }

    // Pomiar per paczka SDK (etap filtra czasowo-przestrzennego).
    struct BatchRecord {
        std::size_t raw_events      = 0;
        std::size_t filtered_events = 0;
        double      filter_us       = 0.0;
    };

    // Pomiar per slice (10 ms) — etapy tracker + TTC oraz metryki jakości.
    struct SliceRecord {
        std::int64_t end_ts_us     = 0;
        std::size_t  slice_events   = 0;
        double       tracker_us     = 0.0;
        double       ttc_us         = 0.0;
        double       slice_total_us = 0.0;   // tracker + ttc
        std::size_t  filter_buffer  = 0;     // temporal_buffer_ filtra
        std::size_t  ttc_history    = 0;     // największe okno historii TTC
        bool         valid          = false;
        bool         danger         = false;
        int          sector         = -1;
        float        ttc_value      = -1.f;  // TTC sektora dominującego [s]
        bool         expanding      = false;
        float        growth_rate    = 0.f;   // px²/s netto w oknie
        float        relative_growth = 0.f;
    };

    explicit PerfMetrics(double slice_budget_us = 10000.0)
        : slice_budget_us_(slice_budget_us) {
        batches_.reserve(4096);
        slices_.reserve(4096);
    }

    void record_batch(std::size_t raw, std::size_t filtered, double filter_us) {
        batches_.push_back(BatchRecord{raw, filtered, filter_us});
    }

    void record_slice(const SliceRecord& r) {
        slices_.push_back(r);
    }

    double slice_budget_us() const noexcept { return slice_budget_us_; }
    std::size_t slice_count() const noexcept { return slices_.size(); }

    // ── Zapis surowych danych per-slice (CSV) ────────────────────────────────
    bool write_csv(const std::string& path) const {
        std::ofstream f(path, std::ios::trunc);
        if (!f.is_open()) {
            return false;
        }
        f << "slice_idx,end_ts_us,slice_events,tracker_us,ttc_us,slice_total_us,"
             "filter_buffer,ttc_history,valid,danger,sector,ttc_value,expanding,"
             "growth_rate,relative_growth\n";
        f << std::fixed << std::setprecision(3);
        for (std::size_t i = 0; i < slices_.size(); ++i) {
            const SliceRecord& s = slices_[i];
            f << i << ',' << s.end_ts_us << ',' << s.slice_events << ','
              << s.tracker_us << ',' << s.ttc_us << ',' << s.slice_total_us << ','
              << s.filter_buffer << ',' << s.ttc_history << ','
              << (s.valid ? 1 : 0) << ',' << (s.danger ? 1 : 0) << ','
              << s.sector << ',' << s.ttc_value << ','
              << (s.expanding ? 1 : 0) << ','
              << s.growth_rate << ',' << s.relative_growth << '\n';
        }
        return true;
    }

    // ── Zapis batch-level CSV (skalowanie filtra) ────────────────────────────
    bool write_batch_csv(const std::string& path) const {
        std::ofstream f(path, std::ios::trunc);
        if (!f.is_open()) {
            return false;
        }
        f << "batch_idx,raw_events,filtered_events,filter_us\n";
        f << std::fixed << std::setprecision(3);
        for (std::size_t i = 0; i < batches_.size(); ++i) {
            const BatchRecord& b = batches_[i];
            f << i << ',' << b.raw_events << ',' << b.filtered_events << ','
              << b.filter_us << '\n';
        }
        return true;
    }

    struct StatSummary {
        double mean = 0, median = 0, p95 = 0, p99 = 0, max = 0, min = 0;
    };

    static StatSummary stats(std::vector<double> v) {
        StatSummary s;
        if (v.empty()) {
            return s;
        }
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (double x : v) sum += x;
        s.mean   = sum / static_cast<double>(v.size());
        s.median = percentile(v, 0.50);
        s.p95    = percentile(v, 0.95);
        s.p99    = percentile(v, 0.99);
        s.min    = v.front();
        s.max    = v.back();
        return s;
    }

    // ── Zbiorcze metryki (liczone na żądanie) ────────────────────────────────
    struct Summary {
        std::size_t total_batches      = 0;
        std::size_t total_slices       = 0;
        std::size_t total_raw_events   = 0;
        std::size_t total_filtered     = 0;
        double      filter_rejection_pct = 0.0;

        StatSummary filter_us;
        StatSummary tracker_us;
        StatSummary ttc_us;
        StatSummary slice_total_us;

        double processing_time_us = 0.0;  // suma wszystkich etapów
        double event_span_us      = 0.0;  // czas zdarzeń pokryty nagraniem
        double throughput_mev_s   = 0.0;  // total_raw / processing_time
        double realtime_factor    = 0.0;  // event_span / processing_time

        double budget_overrun_pct = 0.0;  // % slice'ów > budżet (tracker+ttc)
        std::size_t peak_filter_buffer = 0;
        std::size_t peak_ttc_history   = 0;

        // Jakość detekcji
        std::size_t valid_slices   = 0;
        std::size_t danger_slices  = 0;
        double      detection_rate_pct = 0.0;
        double      danger_rate_pct    = 0.0;
        double      first_detection_latency_s = -1.0;
        std::size_t detection_episodes = 0;
        std::size_t danger_episodes    = 0;
        double      mean_detection_episode_ms = 0.0;
        double      valid_flag_transitions_per_s = 0.0; // jitter detekcji
        double      danger_flag_transitions_per_s = 0.0;
    };

    Summary summarize() const {
        Summary out;
        out.total_batches = batches_.size();
        out.total_slices  = slices_.size();

        std::vector<double> filter_v, tracker_v, ttc_v, total_v;
        filter_v.reserve(batches_.size());
        tracker_v.reserve(slices_.size());
        ttc_v.reserve(slices_.size());
        total_v.reserve(slices_.size());

        double proc = 0.0;
        for (const BatchRecord& b : batches_) {
            out.total_raw_events += b.raw_events;
            out.total_filtered   += b.filtered_events;
            filter_v.push_back(b.filter_us);
            proc += b.filter_us;
        }

        std::size_t overruns = 0;
        for (const SliceRecord& s : slices_) {
            tracker_v.push_back(s.tracker_us);
            ttc_v.push_back(s.ttc_us);
            total_v.push_back(s.slice_total_us);
            proc += s.slice_total_us;
            if (s.slice_total_us > slice_budget_us_) ++overruns;
            out.peak_filter_buffer = std::max(out.peak_filter_buffer, s.filter_buffer);
            out.peak_ttc_history   = std::max(out.peak_ttc_history, s.ttc_history);
            if (s.valid)  ++out.valid_slices;
            if (s.danger) ++out.danger_slices;
        }

        out.filter_us      = stats(filter_v);
        out.tracker_us     = stats(tracker_v);
        out.ttc_us         = stats(ttc_v);
        out.slice_total_us = stats(total_v);
        out.processing_time_us = proc;

        if (out.total_raw_events > 0) {
            out.filter_rejection_pct =
                100.0 * (1.0 - static_cast<double>(out.total_filtered) /
                                static_cast<double>(out.total_raw_events));
        }
        if (proc > 0.0) {
            out.throughput_mev_s =
                static_cast<double>(out.total_raw_events) / proc; // ev/µs = Mev/s
        }
        if (!slices_.empty()) {
            out.event_span_us = static_cast<double>(
                slices_.back().end_ts_us - slices_.front().end_ts_us);
            if (proc > 0.0) {
                out.realtime_factor = out.event_span_us / proc;
            }
            out.budget_overrun_pct =
                100.0 * static_cast<double>(overruns) /
                static_cast<double>(slices_.size());
            out.detection_rate_pct =
                100.0 * static_cast<double>(out.valid_slices) /
                static_cast<double>(slices_.size());
            out.danger_rate_pct =
                100.0 * static_cast<double>(out.danger_slices) /
                static_cast<double>(slices_.size());
        }

        // Jakość: pierwsza detekcja, epizody, jitter
        const std::int64_t first_ts =
            slices_.empty() ? 0 : slices_.front().end_ts_us;
        bool prev_valid = false, prev_danger = false;
        std::size_t valid_transitions = 0, danger_transitions = 0;
        std::int64_t episode_start_ts = 0;
        double episode_ms_sum = 0.0;

        for (std::size_t i = 0; i < slices_.size(); ++i) {
            const SliceRecord& s = slices_[i];
            if (s.valid && out.first_detection_latency_s < 0.0) {
                out.first_detection_latency_s =
                    static_cast<double>(s.end_ts_us - first_ts) * 1e-6;
            }
            if (s.valid && !prev_valid) {
                ++out.detection_episodes;
                episode_start_ts = s.end_ts_us;
            }
            if (!s.valid && prev_valid) {
                episode_ms_sum +=
                    static_cast<double>(s.end_ts_us - episode_start_ts) * 1e-3;
            }
            if (s.valid != prev_valid)   ++valid_transitions;
            if (s.danger && !prev_danger) ++out.danger_episodes;
            if (s.danger != prev_danger) ++danger_transitions;
            prev_valid  = s.valid;
            prev_danger = s.danger;
        }
        if (prev_valid && !slices_.empty()) {
            episode_ms_sum +=
                static_cast<double>(slices_.back().end_ts_us - episode_start_ts) * 1e-3;
        }
        if (out.detection_episodes > 0) {
            out.mean_detection_episode_ms =
                episode_ms_sum / static_cast<double>(out.detection_episodes);
        }
        const double span_s = out.event_span_us * 1e-6;
        if (span_s > 0.0) {
            out.valid_flag_transitions_per_s =
                static_cast<double>(valid_transitions) / span_s;
            out.danger_flag_transitions_per_s =
                static_cast<double>(danger_transitions) / span_s;
        }
        return out;
    }

    bool write_summary_json(const std::string& path) const {
        const Summary s = summarize();
        std::ofstream f(path, std::ios::trunc);
        if (!f.is_open()) {
            return false;
        }
        f << std::fixed << std::setprecision(4);
        auto stat_json = [&](const char* name, const StatSummary& st, bool comma = true) {
            f << "  \"" << name << "\": {"
              << "\"mean\": " << st.mean << ", \"median\": " << st.median
              << ", \"p95\": " << st.p95 << ", \"p99\": " << st.p99
              << ", \"min\": " << st.min << ", \"max\": " << st.max << "}"
              << (comma ? ",\n" : "\n");
        };
        f << "{\n";
        f << "  \"total_batches\": " << s.total_batches << ",\n";
        f << "  \"total_slices\": " << s.total_slices << ",\n";
        f << "  \"total_raw_events\": " << s.total_raw_events << ",\n";
        f << "  \"total_filtered_events\": " << s.total_filtered << ",\n";
        f << "  \"filter_rejection_pct\": " << s.filter_rejection_pct << ",\n";
        f << "  \"slice_budget_us\": " << slice_budget_us_ << ",\n";
        stat_json("filter_us", s.filter_us);
        stat_json("tracker_us", s.tracker_us);
        stat_json("ttc_us", s.ttc_us);
        stat_json("slice_total_us", s.slice_total_us);
        f << "  \"processing_time_us\": " << s.processing_time_us << ",\n";
        f << "  \"event_span_us\": " << s.event_span_us << ",\n";
        f << "  \"throughput_mev_s\": " << s.throughput_mev_s << ",\n";
        f << "  \"realtime_factor\": " << s.realtime_factor << ",\n";
        f << "  \"budget_overrun_pct\": " << s.budget_overrun_pct << ",\n";
        f << "  \"peak_filter_buffer\": " << s.peak_filter_buffer << ",\n";
        f << "  \"peak_ttc_history\": " << s.peak_ttc_history << ",\n";
        f << "  \"valid_slices\": " << s.valid_slices << ",\n";
        f << "  \"danger_slices\": " << s.danger_slices << ",\n";
        f << "  \"detection_rate_pct\": " << s.detection_rate_pct << ",\n";
        f << "  \"danger_rate_pct\": " << s.danger_rate_pct << ",\n";
        f << "  \"first_detection_latency_s\": " << s.first_detection_latency_s << ",\n";
        f << "  \"detection_episodes\": " << s.detection_episodes << ",\n";
        f << "  \"danger_episodes\": " << s.danger_episodes << ",\n";
        f << "  \"mean_detection_episode_ms\": " << s.mean_detection_episode_ms << ",\n";
        f << "  \"valid_flag_transitions_per_s\": " << s.valid_flag_transitions_per_s << ",\n";
        f << "  \"danger_flag_transitions_per_s\": " << s.danger_flag_transitions_per_s << "\n";
        f << "}\n";
        return true;
    }

    void print_summary(std::ostream& os) const {
        const Summary s = summarize();
        os << "\n────────── PODSUMOWANIE WYDAJNOŚCI ──────────\n";
        os << std::fixed << std::setprecision(2);
        os << "Paczki SDK:           " << s.total_batches << "\n";
        os << "Slice'y (10 ms):      " << s.total_slices << "\n";
        os << "Zdarzenia surowe:     " << s.total_raw_events << "\n";
        os << "Zdarzenia po filtrze: " << s.total_filtered
           << "  (odrzucono " << s.filter_rejection_pct << "%)\n";
        os << "Throughput:           " << s.throughput_mev_s << " Mev/s\n";
        os << "Real-time factor:     " << s.realtime_factor << "x\n";
        os << "Czas przetwarzania:   " << s.processing_time_us * 1e-3 << " ms\n";
        os << "\nLatencja [µs]         mean / median / p95 / p99 / max\n";
        auto line = [&](const char* n, const StatSummary& st) {
            os << "  " << std::left << std::setw(18) << n << std::right
               << st.mean << " / " << st.median << " / " << st.p95
               << " / " << st.p99 << " / " << st.max << "\n";
        };
        line("filtr (paczka)", s.filter_us);
        line("tracker (slice)", s.tracker_us);
        line("ttc (slice)", s.ttc_us);
        line("tracker+ttc", s.slice_total_us);
        os << "Slice'y > budżet 10ms: " << s.budget_overrun_pct << "%\n";
        os << "Szczyt bufora filtra:  " << s.peak_filter_buffer << " zdarzeń\n";
        os << "Szczyt historii TTC:   " << s.peak_ttc_history << " próbek\n";
        os << "\n── Jakość detekcji ──\n";
        os << "Detekcje (valid):     " << s.valid_slices
           << "  (" << s.detection_rate_pct << "% slice'ów)\n";
        os << "Kolizje (danger):     " << s.danger_slices
           << "  (" << s.danger_rate_pct << "% slice'ów)\n";
        os << "Pierwsza detekcja:    " << s.first_detection_latency_s << " s\n";
        os << "Epizody detekcji:     " << s.detection_episodes
           << "  (śr. " << s.mean_detection_episode_ms << " ms)\n";
        os << "Epizody kolizji:      " << s.danger_episodes << "\n";
        os << "Jitter detekcji:      " << s.valid_flag_transitions_per_s << " zmian/s\n";
        os << "─────────────────────────────────────────────\n";
    }

private:
    static double percentile(const std::vector<double>& sorted, double q) {
        if (sorted.empty()) return 0.0;
        if (sorted.size() == 1) return sorted.front();
        const double idx = q * static_cast<double>(sorted.size() - 1);
        const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
        const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
        const double frac = idx - static_cast<double>(lo);
        return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
    }

    double slice_budget_us_;
    std::vector<BatchRecord> batches_;
    std::vector<SliceRecord> slices_;
};

} // namespace oa
