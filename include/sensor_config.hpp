#pragma once
#include <cstdint>

namespace oa::config {

inline constexpr int DEFAULT_SENSOR_W = 320;
inline constexpr int DEFAULT_SENSOR_H = 320;

/// Okno przetwarzania trackera/TTC — wspólne dla live i replay.
inline constexpr int64_t SLICE_DURATION_US = 10000; // 10 ms

/// Okno filtra czasowo-przestrzennego.
inline constexpr int64_t TEMPORAL_WINDOW_US = 3000; // 3 ms — więcej kontekstu dla sąsiadów

/// Kalibracja tła: pierwsze N slice'ów tylko obserwacja.
inline constexpr int CALIBRATION_SLICES = 30;

/// Dolne progi adaptacyjne (nie schodzimy poniżej).
inline constexpr int MIN_DETECTION_THRESH    = 35;
inline constexpr int MIN_SECTOR_MIN_EVENTS   = 10;

/// Ułamek zdarzeń w slice/sektorze — łagodzi progi przy słabszym sygnale.
inline constexpr int SLICE_EVENT_RATIO_PCT   = 12;  // % całego slice
inline constexpr int SECTOR_EVENT_RATIO_PCT  = 8;   // % slice w sektorze dominującym

} // namespace oa::config
