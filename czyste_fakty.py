import numpy as np
from metavision_core.event_io import EventsIterator

"""
Kod dedykowany pod Raspberry (jak się łączę przez SSH) do monitorowania sceny w czasie rzeczywistym.
Zamiast zapisywać dane do pliku, odczytujemy je bezpośrednio z kamery USB i analizujemy w locie, aby wykrywać aktywne obiekty.
Każda paczka zdarzeń jest filtrowana. Jeśli wykryjemy wystarczającą liczbę czystych zdarzeń (po usunięciu hot pixels), 
wypisujemy raport o wykrytym ruchu, rozmiarze obiektu i jego pozycji.
"""


# Pusty string odpala kamerę na żywo przez USB
INPUT_SOURCE = ""  
DT_US = 40000  # Okno 40 ms
width, height = 320, 320

print("Uruchamianie detektora SSH na żywo...")
try:
    mv_iterator = EventsIterator(input_path=INPUT_SOURCE, delta_t=DT_US)
except Exception as e:
    print(f"Błąd: Brak kamery na USB 3.0? Szczegóły: {e}")
    exit(1)

print("\n[SYSTEM AKTYWNY] Monitorowanie sceny przez SSH... (Wciśnij Ctrl+C aby przerwać)")

for evs in mv_iterator:
    # Jeśli w 40ms nic się nie ruszyło, idź dalej
    if evs.size == 0:
        continue
        
    x_raw = evs['x'].astype(int)
    y_raw = evs['y'].astype(int)
    
    # 1. Filtracja szumu (Hot Pixels)
    temp_map = np.zeros((height, width))
    np.add.at(temp_map, (y_raw, x_raw), 1)
    clean_mask = temp_map[y_raw, x_raw] <= 5
    
    # Zliczamy ile czystych zdarzeń zarejestrowano w tej paczce
    active_points = np.sum(clean_mask)
    
    # 2. PRÓG DETEKCJI: Jeśli czystych punktów jest więcej niż np. 300, 
    # to znaczy, że to nie jest losowy szum, tylko realny obiekt!
    if active_points > 300:
        x_clean = x_raw[clean_mask]
        y_clean = y_raw[clean_mask]
        
        # Wyznaczamy rozmiar obiektu (Bounding Box)
        min_x, max_x = np.min(x_clean), np.max(x_clean)
        min_y, max_y = np.min(y_clean), np.max(y_clean)
        obj_width = max_x - min_x
        obj_height = max_y - min_y
        
        # Wypisujemy raport na żywo w jednej linii terminala (\r podmienia tekst)
        print(f"\r[ALERT] Wykryto ruch! Punkty: {active_points} | Rozmiar: {obj_width}x{obj_height} px | Pozycja X: {min_x}-{max_x}", end="", flush=True)