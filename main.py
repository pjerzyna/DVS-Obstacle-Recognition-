import numpy as np
import matplotlib.pyplot as plt
from metavision_core.event_io import EventsIterator

"""
Ten plik przedstawia Proof of Concept (PoC) dla przetwarzania strumienia 
zdarzeń z kamery GenX320 w czasie rzeczywistym, bezpośrednio z pamięci RAM.
Zamiast zapisywać dane do pliku, odczytujemy je paczkami po 40 ms (40 000 mikrosekund)
 i natychmiast je przetwarzamy, tworząc dynamiczny obraz zdarzeń na żywo.
Każda paczka jest filtrowana w locie, aby usunąć hot pixels 
(piksele, które strzelają zbyt często) i akumulowana do obrazu IWE,  
który jest aktualizowany na ekranie w czasie rzeczywistym.    
"""


# =====================================================================
# CONFIGURATION - PRZEŁĄCZNIK TRYBU PRACY
# =====================================================================
# TRYB PLIKU (PC): Podaj ścieżkę do pliku .raw, np. "data_in/data.raw"
# TRYB NA ŻYWO (Raspberry Pi 5): Ustaw INPUT_SOURCE = "" (pusty string wykryje kamerę USB)
INPUT_SOURCE = "data_in/naprawiony_ruch.raw"  

DT_US = 40000  # Wielkość okna: 40 milisekund (idealne dla ludzkiego ruchu)
width, height = 320, 320 # Rozdzielczość sensora GenX320

print("Inicjalizacja strumienia danych...")
try:
    mv_iterator = EventsIterator(input_path=INPUT_SOURCE, delta_t=DT_US)
except Exception as e:
    print(f"Błąd inicjalizacji! Sprawdź połączenie lub ścieżkę. Szczegóły: {e}")
    exit(1)

# Przygotowanie okna wyświetlania (Zredukowane do jednego, czystego panelu)
plt.ion()
fig, ax = plt.subplots(figsize=(7, 7))
im = ax.imshow(np.zeros((height, width)), cmap='hot', vmin=0, vmax=3)
ax.set_title("Detekcja Aktywnych Obiektów (PoC)")
plt.colorbar(im, ax=ax, label='Intensywność ruchu (Liczba zdarzeń)')

print("\nSystem uruchomiony. Przetwarzanie strumienia w pamięci RAM...")

for evs in mv_iterator:
    # Jeśli brak ruchu w ciągu 40ms, wyczyść ekran i idź dalej
    if evs.size == 0:
        im.set_data(np.zeros((height, width)))
        plt.pause(0.001)
        continue
        
    # Wyciągamy surowe współrzędne z binarnej paczki RAM
    x_raw = evs['x'].astype(int)
    y_raw = evs['y'].astype(int)
    
    # -----------------------------------------------------------------
    # KROK 1: SPRZĘTOWA FILTRACJA SZUMU (HOT PIXELS)
    # -----------------------------------------------------------------
    # Zliczamy aktywność pikseli w tym krótkim oknie
    temp_map = np.zeros((height, width))
    np.add.at(temp_map, (y_raw, x_raw), 1)
    
    # Jeśli piksel w 40ms strzelił więcej niż 5 razy -> usuwamy go jako szum tła
    clean_mask = temp_map[y_raw, x_raw] <= 5
    
    # -----------------------------------------------------------------
    # KROK 2: GENEROWANIE RAMKI DETEKCJI
    # -----------------------------------------------------------------
    iwe_detection = np.zeros((height, width))
    np.add.at(iwe_clean := np.zeros((height, width)), (y_raw[clean_mask], x_raw[clean_mask]), 1)
    
    # -----------------------------------------------------------------
    # KROK 3: WYŚWIETLANIE (Wizualna weryfikacja sukcesu)
    # -----------------------------------------------------------------
    im.set_data(iwe_clean)
    
    current_time_sec = evs['t'][0] / 1_000_000.0
    ax.set_title(f"Detektor RPi 5 | Czas strumienia: {current_time_sec:.2f} s\nAktywne punkty: {np.sum(clean_mask)}")
    
    plt.pause(0.001)
    
    if not plt.fignum_exists(fig.number):
        break

plt.ioff()
print("\nStrumień zakończony.")