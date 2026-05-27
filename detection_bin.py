import cv2  # Używany wyłącznie do matematycznej filtracji 2D
import numpy as np
import time  

# =====================================================================
# CONFIGURATION
# =====================================================================
INPUT_SOURCE = "data_in/wykryty_ruch.bin"
IMAGE_SHAPE = (320, 320)  # Dostosowane do GenX320

# --- SZYBKIE GLOBALNE WCZYTANIE DANYCH BINARNYCH ---
print("Wczytywanie całego pliku binarnego do pamięci RAM...")

# Oficjalna struktura 16-bajtowa Metavision
EVENT_DTYPE = np.dtype({
    'names': ['x', 'y', 'p', 't'],
    'formats': ['<u2', '<u2', '<i2', '<i8'],
    'offsets': [0, 2, 4, 8],
    'itemsize': 16
})

try:
    raw_data = np.fromfile(INPUT_SOURCE, dtype=EVENT_DTYPE)
    print(f"Pomyślnie wczytano {len(raw_data)} zdarzeń z pliku binarnego.")
except Exception as e:
    print(f"Błąd wczytywania pliku binarnego: {e}")
    exit(1)

# Konwersja mikrosekund na sekundy i wyzerowanie startu nagrania
t_seconds = (raw_data['t'] - raw_data['t'][0]) / 1_000_000.0

# Globalne maskowanie (okno 1 do 5 sekundy nagrania)
global_mask = (t_seconds >= 1.0) & (t_seconds <= 5.0)
data_filtered = raw_data[global_mask]

timestamps = t_seconds[global_mask]
xs = data_filtered['x'].astype(np.int32)
ys = data_filtered['y'].astype(np.int32)

print(f"Zdarzenia po odfiltrowaniu okna 1-25s: {len(timestamps)}")

def main():
    H, W = IMAGE_SHAPE
    h_step, w_step = H // 3, W // 3
    sector_area = h_step * w_step

    tau = 0.01  # Okno analizy: 10 ms
    t_start = 1.0
    t_max = 25.0

    frame = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
    kernel_weights = np.ones((3, 3), np.uint8)

    # Bufor do wyliczenia czystej latencji rdzenia algorytmu
    algo_latencies = []

    print("\nUruchamianie sektorowego detektora (Tryb czystej wydajności)...")
    while t_start < t_max:
        t_end = t_start + tau

        # --- POMIAR: START SEKCJI ALGORYTMICZNEJ ---
        t_algo_start = time.perf_counter()

        t_mask = (timestamps >= t_start) & (timestamps < t_end)
        x_w = xs[t_mask]
        y_w = ys[t_mask]
        
        if len(x_w) == 0: 
            t_start = t_end
            continue

        frame.fill(0)
        frame[y_w, x_w] = 255  

        # Filtrowanie szumu (Działa w 100% bez interfejsu graficznego)
        neighbor_count = cv2.filter2D(frame, -1, kernel_weights)
        cleaned = np.where((frame == 255) & (neighbor_count >= 3), 255, 0).astype(np.uint8)

        # Analiza siatki sektorów 3x3
        for row in range(3):
            for col in range(3):
                y_start, y_end = row * h_step, (row + 1) * h_step
                x_start, x_end = col * w_step, (col + 1) * w_step

                sector = cleaned[y_start:y_end, x_start:x_end]
                edge_pixels = np.sum(sector == 255)
                density_percentage = (edge_pixels / sector_area) * 100

        # --- POMIAR: KONIEC SEKCJI ALGORYTMICZNEJ ---
        t_algo_end = time.perf_counter()
        
        current_algo_latency = (t_algo_end - t_algo_start) * 1000
        algo_latencies.append(current_algo_latency)

        t_start = t_end

    # --- PODSUMOWANIE STATYSTYK ---
    if algo_latencies:
        print("\n=== RAPORT OPÓŹNIEŃ PRZETWARZANIA (CORE PERFORMANCE) ===")
        print(f"Średnia latencja algorytmiczna (Core): {np.mean(algo_latencies):.3f} ms")
        print(f"Maksymalna latencja algorytmiczna (Worst-case): {np.max(algo_latencies):.3f} ms")
        print(f"Teoretyczna wydajność potoku: {1000.0 / np.mean(algo_latencies):.1f} klatek/s (FPS)")
        print("========================================================")

if __name__ == "__main__":
    main()