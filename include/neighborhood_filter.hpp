#pragma once
#include <metavision/sdk/base/events/event_cd.h>
#include <vector>
#include <cstdint>

namespace oa {

// ─────────────────────────────────────────────────────────────────────────────
//  NeighborhoodFilter — 4-kierunkowy filtr sąsiedztwa w oknie czasowym.
//
//  Zdarzenie przeżywa, jeśli ≥ MIN_NEIGHBORS sąsiadów (N/S/E/W) było
//  aktywnych w tej samej paczce DT_US. Eliminuje hot-pixels i szum termiczny.
//
//  Optymalizacje względem wersji bazowej:
//  1. Selektywne czyszczenie mapy (O(N_events) zamiast O(320*320) memset).
//     Przy typowych ~2000 eventach → 50x mniej operacji zapisu.
//  2. __builtin_prefetch na 8 kroków do przodu → ukrywa latencję L2 (12 cykli).
//  3. __restrict__ na wskaźnikach → usuwa aliasing, pozwala kompilatorowi
//     agresywniej optymalizować pętle build_map i filter.
//  4. resize+indeks zamiast push_back → zero sprawdzeń capacity w pętli.
//
//  KONTRAKT WĄTKOWY: obiekt jest własnością wyłącznie wątku callbacku SDK.
//  NIE modyfikować z wątku main() bez mutex<>.
// ─────────────────────────────────────────────────────────────────────────────
class NeighborhoodFilter {
public:
    static constexpr int SENSOR_W     = 320;
    static constexpr int SENSOR_H     = 320;
    static constexpr int MIN_NEIGHBORS = 2;

    explicit NeighborhoodFilter();

    /// Filtruje events_in → events_out.
    /// Zwraca liczbę zdarzeń, które przeżyły filtrację.
    size_t filter(
        const Metavision::EventCD* __restrict__ events_in,
        size_t                                   count,
        std::vector<Metavision::EventCD>&        events_out
    );

private:
    /// Mapa zajętości pikseli bieżącej paczki.
    /// alignas(16) → NEON wymaga wyrównania do 16 B dla vld1q_u8.
    /// uint8_t zamiast bool → bezpieczna arytmetyka sąsiadów bez branch.
    alignas(16) uint8_t event_map_[SENSOR_H * SENSOR_W];

    /// Zeruje wyłącznie piksele zajęte w bieżącej paczce (O(N_events)).
    /// Wywoływana na KOŃCU filter() — mapa jest wtedy "ciepła" w cache L1.
    void selective_clear(const Metavision::EventCD* __restrict__ events, size_t count);

    void build_map(const Metavision::EventCD* __restrict__ events, size_t count);
};

} // namespace oa
