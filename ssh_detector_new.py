import numpy as np
import sys
from metavision_core.event_io import EventsIterator

"""
KOD NA RASPBERRY PI 5 PRZEZ SSH
"""

INPUT_SOURCE = ""  
DT_US = 40000  
width, height = 320, 320
OUTPUT_FILE = "wykryty_ruch.bin"  # Zmiana rozszerzenia dla jasności

print("Uruchamianie produkcyjnego trackera SSH...")
try:
    mv_iterator = EventsIterator(input_path=INPUT_SOURCE, delta_t=DT_US)
except Exception as e:
    print(f"Błąd inicjalizacji: {e}")
    exit(1)

# Sprzętowa modyfikacja czułości sensora GenX320
try:
    device = mv_iterator.reader.device
    if device is not None:
        i_ll_biases = device.get_i_ll_biases()
        i_ll_biases.set("bias_diff_on", 45)
        i_ll_biases.set("bias_diff_off", 45)
        print("-> Czułość sprzętowa zaaplikowana pomyślnie!")
except Exception as bias_err:
    print(f"[OSTRZEŻENIE] Problem z biases: {bias_err}")

print(f"-> Przygotowanie binarnego pliku wyjściowego: {OUTPUT_FILE}")
raw_file = open(OUTPUT_FILE, "wb")

print("\n[SYSTEM TRACKINGU I REJESTRACJI AKTYWNY]")
print("Wciśnij Ctrl+C, aby zakończyć i zapisać.\n")

prev_cx, prev_cy = None, None
MOVEMENT_THRESHOLD = 3.0  

try:
    for evs in mv_iterator:
        if evs.size == 0:
            continue
            
        x_raw = evs['x'].astype(int)
        y_raw = evs['y'].astype(int)
        
        # Filtracja (Twój sprawdzony kod)
        event_map = np.zeros((height, width), dtype=np.uint8)
        event_map[y_raw, x_raw] = 1
        
        neighbors = (
            np.roll(event_map,  1, axis=0) +
            np.roll(event_map, -1, axis=0) +
            np.roll(event_map,  1, axis=1) +
            np.roll(event_map, -1, axis=1)
        )
        clean_mask_2d = (event_map == 1) & (neighbors >= 2)
        clean_mask_events = clean_mask_2d[y_raw, x_raw]
        
        x_clean = x_raw[clean_mask_events]
        y_clean = y_raw[clean_mask_events]
        active_points = len(x_clean)
        
        # =================================================================
        # ZAPIS BINARNY (Szybki i bezpieczny dla RPi)
        # =================================================================
        if active_points > 0:
            raw_file.write(evs[clean_mask_events].tobytes())
        
        # Logika wyświetlania w konsoli SSH
        if active_points > 150:
            cx = int(np.mean(x_clean))
            cy = int(np.mean(y_clean))
            direction_str = "STACJONARNY"
            if prev_cx is not None and prev_cy is not None:
                dx = cx - prev_cx
                dy = cy - prev_cy
                if abs(dx) > abs(dy) and abs(dx) > MOVEMENT_THRESHOLD:
                    direction_str = "W PRAWO" if dx > 0 else "W LEWO"
                elif abs(dy) > abs(dx) and abs(dy) > MOVEMENT_THRESHOLD:
                    direction_str = "W DÓŁ" if dy > 0 else "W GÓRĘ"
            
            prev_cx, prev_cy = cx, cy
            print(f"\r\033[K[NAGRYWANIE] Środek: ({cx:3d}, {cy:3d}) | Kierunek: {direction_str:11s} | Punkty: {active_points:5d}", end="", flush=True)
        else:
            prev_cx, prev_cy = None, None
            print(f"\r\033[K[STATUS] Scena czysta (Zdarzenia tła: {np.sum(event_map):4d})", end="", flush=True)

except KeyboardInterrupt:
    print("\n\nZamykanie strumienia...")

finally:
    if 'raw_file' in locals():
        raw_file.flush()
        raw_file.close()
        print(f"-> SUKCES: Dane zapisano w: {OUTPUT_FILE}")
        
    if 'mv_iterator' in locals():
        del mv_iterator