import numpy as np
import sys
from metavision_core.event_io import EventsIterator

"""
Docelowo ten plik będzie uruchamiany na Raspberry Pi 5 przez SSH, bezpośrednio z podłączoną kamerą GenX320.
Zamiast zapisywać dane do pliku, odczytujemy je bezpośrednio z kamery USB i analizujemy w locie, aby wykrywać aktywne obiekty.
Każda paczka zdarzeń jest filtrowana. Jeśli wykryjemy wystarczającą liczbę czystych zdarzeń. - to jest pierwowzór do dalszej rozbudowy kodu
"""

INPUT_SOURCE = ""  
DT_US = 40000  
width, height = 320, 320

print("Uruchamianie produkcyjnego detektora SSH...")
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
        print("-> Czułość sprzętowa zaaplikowana (Szum wycięty u źródła)!")
except Exception as bias_err:
    print(f"[OSTRZEŻENIE] Problem z biases: {bias_err}")

print("\n[SYSTEM MONITOROWANIA EMBEDDED AKTYWNY]")
print("Wyjście z programu: Ctrl+C\n")

try:
    for evs in mv_iterator:
        if evs.size == 0:
            continue
            
        x_raw = evs['x'].astype(int)
        y_raw = evs['y'].astype(int)
        
        # 1. Mapa zdarzeń
        event_map = np.zeros((height, width), dtype=np.uint8)
        event_map[y_raw, x_raw] = 1
        
        # 2. Lekki, 4-kierunkowy filtr sąsiedztwa (idealny pod RPi 5 CPU)
        neighbors = (
            np.roll(event_map,  1, axis=0) +
            np.roll(event_map, -1, axis=0) +
            np.roll(event_map,  1, axis=1) +
            np.roll(event_map, -1, axis=1)
        )
        clean_map = (event_map == 1) & (neighbors >= 2)
        
        y_clean, x_clean = np.where(clean_map)
        active_points = len(x_clean)
        
        # 3. Kryterium detekcji (wartość 150 punktów strukturalnych)
        # Sekwencja \033[K na końcu czyści stare znaki w terminalu
        if active_points > 150:
            min_x, max_x = np.min(x_clean), np.max(x_clean)
            min_y, max_y = np.min(y_clean), np.max(y_clean)
            obj_width = max_x - min_x
            obj_height = max_y - min_y
            
            print(f"\r\033[K[ALERT - DETEKCJA] Obiekt aktywny! Punkty: {active_points:5d} | Rozmiar: {obj_width:3d}x{obj_height:3d} px", end="", flush=True)
        else:
            print(f"\r\033[K[STATUS] Scena czysta (Zdarzenia tła: {np.sum(event_map):4d})", end="", flush=True)

except KeyboardInterrupt:
    print("\n\nZamykanie systemu detekcji przez użytkownika.")

finally:
    if 'mv_iterator' in locals():
        del mv_iterator 
    print("Zasoby zwolnione. Kamera gotowa do kolejnych zadań.")
