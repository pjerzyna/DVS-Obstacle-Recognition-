#pragma once
#include <metavision/sdk/stream/camera.h>
#include <cstdint>

namespace oa {

/// Sprzętowa konfiguracja rejestrów czułości sensora GenX320.
/// Ucisza matrycę termicznie: bez tej konfiguracji >60k noise events/40ms.
class BiasConfigurator {
public:
    /// Wartości wyznaczone empirycznie w fazie PoC.
    static constexpr int32_t DIFF_ON_VALUE  = 45;
    static constexpr int32_t DIFF_OFF_VALUE = 45;

    /// Aplikuje biasy na urządzeniu. Zwraca true przy sukcesie.
    static bool apply(Metavision::Camera& camera);
};

} // namespace oa
