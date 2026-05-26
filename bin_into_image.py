import numpy as np
import matplotlib.pyplot as plt

# =====================================================================
# CONFIGURATION
# =====================================================================
INPUT_SOURCE = "data_in/wykryty_ruch.bin"  # lub .raw, zależnie jak nazwałeś plik
DT_US = 40000  # Wielkość okna: 40 ms
width, height = 320, 320

# --- TUTAJ JEST KLUCZOWA ZMIANA: PEŁNY I OFICJALNY DTYPE METAVISION ---
EVENT_DTYPE = np.dtype({
    'names': ['x', 'y', 'p', 't'],
    'formats': ['<u2', '<u2', '<i2', '<i8'],
    'offsets': [0, 2, 4, 8],
    'itemsize': 16
})

print("Wczytywanie całego pliku binarnego...")
try:
    all_events = np.fromfile(INPUT_SOURCE, dtype=EVENT_DTYPE)
    print(f"Pomyślnie wczytano {len(all_events)} zdarzeń.")
except Exception as e:
    print(f"Błąd odczytu pliku! Szczegóły: {e}")
    exit(1)

if len(all_events) == 0:
    print("Plik jest pusty!")
    exit()

# Przygotowanie wykresu
plt.ion()
fig, ax = plt.subplots(figsize=(7, 7))
im = ax.imshow(np.zeros((height, width)), cmap='hot', vmin=0, vmax=3)
ax.set_title("Odtwarzanie przefiltrowanego strumienia")
plt.colorbar(im, ax=ax, label='Liczba zdarzeń')

start_time = all_events['t'][0]
end_time = all_events['t'][-1]
current_time = start_time

print(f"Wykryty zakres czasu: {start_time} us do {end_time} us")
print("\nRozpoczynam wyświetlanie...")

while current_time < end_time:
    # Wycinanie zdarzeń z okna 40 ms
    mask = (all_events['t'] >= current_time) & (all_events['t'] < current_time + DT_US)
    evs = all_events[mask]
    
    if evs.size == 0:
        im.set_data(np.zeros((height, width)))
        plt.pause(0.001)
        current_time += DT_US
        continue
        
    x_raw = evs['x'].astype(int)
    y_raw = evs['y'].astype(int)
    
    # Tworzenie klatki do wyświetlenia
    iwe = np.zeros((height, width))
    np.add.at(iwe, (y_raw, x_raw), 1)
    
    # Aktualizacja okna
    im.set_data(iwe)
    current_time_sec = (current_time - start_time) / 1_000_000.0
    ax.set_title(f"Czas: {current_time_sec:.2f} s | Zdarzeń w klatce: {len(evs)}")
    
    plt.pause(0.01)  # Dajemy Matplotlibowi czas na wyrysowanie ramki
    
    if not plt.fignum_exists(fig.number):
        break
        
    current_time += DT_US

plt.ioff()
print("\nKoniec pliku.")
plt.show(block=True)  # Zatrzymuje okno na ekranie po zakończeniu filmu