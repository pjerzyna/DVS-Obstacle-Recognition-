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

# Ścieżka do pliku binarnego RAW
INPUT_RAW = "data_in/data.raw"

# Parametry okna czasowego (paczki po 40 ms = 40 000 mikrosekund)
DT_US = 40000 

print(f"Inicjalizacja strumienia dla pliku: {INPUT_RAW}")
# delta_t w EventsIterator definiuje, jak duże paczki czasowe dostajemy w każdej iteracji
mv_iterator = EventsIterator(input_path=INPUT_RAW, delta_t=DT_US)

# Przygotowanie interaktywnego okna Matplotlib do wyświetlania animacji na żywo
plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))
# Tworzymy pustą matrycę 320x320, vmin/vmax blokuje kontrast, żeby obraz nie migał
im = ax.imshow(np.zeros((320, 320)), cmap='hot', vmin=0, vmax=3)
ax.set_title("Strumień w czasie rzeczywistym (Paczki 40ms)\nProof of Concept")
plt.colorbar(im, ax=ax, label='Liczba zdarzeń')
print("\nRozpoczynam przetwarzanie paczek w pamięci RAM...")

# GŁÓWNA PĘTLA STRUMIENIOWA 
for evs in mv_iterator:
    if evs.size == 0:
        continue
        
    # Wyciągamy surowe współrzędne z binarnej paczki RAM
    x_slice = evs['x'].astype(int)
    y_slice = evs['y'].astype(int)
    
    # -------------------------------------------------------------
    # KROK A: FILTRACJA SZUMU (HOT PIXELS) W LOCIE
    # -------------------------------------------------------------
    # Zliczamy wystąpienia pikseli w tej konkretnej paczce
    temp_map = np.zeros((320, 320))
    np.add.at(temp_map, (y_slice, x_slice), 1)
    # Jeśli piksel strzelił więcej niż 5 razy w ciągu 40ms -> usuwamy go jako szum
    clean_mask = temp_map[y_slice, x_slice] <= 5
    
    # -------------------------------------------------------------
    # KROK B: AKUMULACJA CZYSZCZONEGO OBRAZU
    # -------------------------------------------------------------
    iwe_clean = np.zeros((320, 320))
    np.add.at(iwe_clean, (y_slice[clean_mask], x_slice[clean_mask]), 1)
    
    # -------------------------------------------------------------
    # KROK C: AKTUALIZACJA EKRANU (ANIMACJA NA ŻYWO)
    # -------------------------------------------------------------
    im.set_data(iwe_clean)
    # Pobieramy aktualny czas z paczki, żeby wyświetlić go w tytule (zamiana us na sekundy)
    current_time_sec = evs['t'][0] / 1_000_000.0
    ax.set_title(f"Strumień Paczek 40ms \nCzas wideo: {current_time_sec:.2f} s")
    # Wymuszenie odświeżenia rysunku w Matplotlib (krótka pauza generuje efekt płynnego filmu)
    plt.pause(0.01)
    
    # Opcjonalne zabezpieczenie: zatrzymaj, jeśli zamkniesz okienko
    if not plt.fignum_exists(fig.number):
        break

plt.ioff()
print("\nPrzetwarzanie strumienia zakończone.")