#pragma once
#include <metavision/sdk/base/events/event_cd.h>
#include <vector>

namespace oa {

// ─────────────────────────────────────────────────────────────────────────────
//  Direction — enum zamiast std::string w hot path.
//  Eliminuje alokację na stercie przy każdej klatce (25x/s = 25 malloc/free).
//  Konwersja do C-string tylko przy wyświetlaniu w konsoli.
// ─────────────────────────────────────────────────────────────────────────────
enum class Direction : uint8_t {
    STATIONARY = 0,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

/// Pomocnicza — wywoływana wyłącznie przy wyświetlaniu, nie w hot path.
inline const char* direction_to_cstr(Direction d) noexcept {
    switch (d) {
        case Direction::LEFT:       return "W LEWO";
        case Direction::RIGHT:      return "W PRAWO";
        case Direction::UP:         return "W GORĘ";
        case Direction::DOWN:       return "W DÓŁ";
        case Direction::STATIONARY: // fall-through
        default:                    return "STACJONARNY";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  TrackResult — wynik analizy jednej paczki zdarzeń (40 ms).
//  Brak std::string — cała struktura mieści się w jednej linii cache (64 B).
// ─────────────────────────────────────────────────────────────────────────────
struct TrackResult {
    // Geometria przeszkody
    int cx      = 0, cy      = 0;  // Centroid
    int bbox_x1 = 0, bbox_y1 = 0;  // Bounding Box lewy-górny
    int bbox_x2 = 0, bbox_y2 = 0;  // Bounding Box prawy-dolny
    int bbox_w  = 0, bbox_h  = 0;  // Wymiary BB [px]

    // Kinematyka
    int       dx = 0, dy = 0;      // Δ centroidu względem poprzedniej klatki
    Direction direction = Direction::STATIONARY;

    // Sektory: 0 = LEWY, 1 = CENTRALNY, 2 = PRAWY
    int    dominant_sector = 1;
    size_t active_points   = 0;
    bool   valid           = false; // false jeśli < DETECTION_THRESH punktów
};

// ─────────────────────────────────────────────────────────────────────────────
//  ObstacleTracker — wyznacza BBox, Centroid i kierunek ruchu.
//  Bezstanowy względem SDK — trzyma tylko geometrię poprzedniej klatki.
//
//  KONTRAKT WĄTKOWY: obiekt jest własnością wyłącznie wątku callbacku SDK.
//  NIE modyfikować z wątku main() bez mutex<>.
// ─────────────────────────────────────────────────────────────────────────────
class ObstacleTracker {
public:
    static constexpr int   SENSOR_W           = 320;
    static constexpr int   DETECTION_THRESH   = 150;
    static constexpr int   MOVEMENT_THRESH    = 3;   // int, nie float — unika FP w hot path

    // Podział sensora na 3 równe sektory (320 / 3 ≈ 107 px)
    static constexpr int   SECTOR_LEFT_END    = 106;
    static constexpr int   SECTOR_RIGHT_START = 214;

    explicit ObstacleTracker() = default;

    TrackResult process(const std::vector<Metavision::EventCD>& events);

private:
    int prev_cx_ = -1, prev_cy_ = -1;
};

} // namespace oa
