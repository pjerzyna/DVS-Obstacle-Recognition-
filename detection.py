import cv2
import numpy as np
import time  

# --- SZYBKIE GLOBALNE WCZYTANIE DANYCH  ---
print("Wczytywanie całego pliku zdarzeń do pamięci RAM...")
data = np.loadtxt('events.txt')

global_mask = (data[:, 0] >= 1.0) & (data[:, 0] <= 5.0)
data_filtered = data[global_mask]

timestamps = data_filtered[:, 0]
xs = data_filtered[:, 1].astype(np.int32)
ys = data_filtered[:, 2].astype(np.int32)

def main():
    image_shape = (180, 240)
    H, W = image_shape
    h_step, w_step = H // 3, W // 3
    sector_area = h_step * w_step

    tau = 0.01  # 10 ms
    t_start = 1.0
    t_max = 5.0

    frame = np.zeros(image_shape, dtype=np.uint8)
    kernel_weights = np.ones((3, 3), np.uint8)

    # Bufory do wyliczenia statystyk globalnych
    algo_latencies = []
    gui_latencies = []

    print("Uruchamianie sektorowego detektora z profilerem latencji...")
    while t_start < t_max:
        t_end = t_start + tau

        # --- POMIAR 1: START SEKCJI ALGORYTMICZNEJ ---
        t_algo_start = time.perf_counter()

        t_mask = (timestamps >= t_start) & (timestamps < t_end)
        x_w = xs[t_mask]
        y_w = ys[t_mask]
        
        if len(x_w) == 0: 
            t_start = t_end
            continue

        frame.fill(0)
        frame[y_w, x_w] = 255  

        # Filtrowanie szumu
        neighbor_count = cv2.filter2D(frame, -1, kernel_weights)
        cleaned = np.where((frame == 255) & (neighbor_count >= 3), 255, 0).astype(np.uint8)

        # Analiza siatki 3x3
        densities = []
        for row in range(3):
            for col in range(3):
                y_start, y_end = row * h_step, (row + 1) * h_step
                x_start, x_end = col * w_step, (col + 1) * w_step

                sector = cleaned[y_start:y_end, x_start:x_end]
                edge_pixels = np.sum(sector == 255)
                density_percentage = (edge_pixels / sector_area) * 100
                densities.append((x_start, y_start, x_end, y_end, density_percentage))

        # --- POMIAR 1: KONIEC SEKCJI ALGORYTMICZNEJ ---
        t_algo_end = time.perf_counter()
        
        # Obliczenie czystej latencji algorytmu dla obecnej klatki (w milisekundach)
        current_algo_latency = (t_algo_end - t_algo_start) * 1000
        algo_latencies.append(current_algo_latency)

        # --- POMIAR 2: START SEKCJI WIZUALIZACJI (OpenCV GUI) ---
        t_gui_start = time.perf_counter()

        vis = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
        for x_start, y_start, x_end, y_end, density in densities:
            color = (0, 0, 255) if density > 5.0 else (0, 255, 0)
            cv2.rectangle(vis, (x_start, y_start), (x_end, y_end), color, 1)
            cv2.putText(vis, f"{density:.1f}%", (x_start + 10, y_start + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Dodatkowe wyświetlanie obecnej latencji na ekranie
        cv2.putText(vis, f"Latency: {current_algo_latency:.2f} ms", (10, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.imshow("Sektorowy Detektor Przeszkod 3x3", vis)
        
        # Krok waitKey(1) przerywa pętlę i wymusza odświeżenie okna GUI
        key = cv2.waitKey(1)
        
        t_gui_end = time.perf_counter()
        current_gui_latency = (t_gui_end - t_gui_start) * 1000
        gui_latencies.append(current_gui_latency)

        if key == 27: 
            break

        t_start = t_end

    cv2.destroyAllWindows()

    # --- PODSUMOWANIE STATYSTYK ---
    if algo_latencies:
        print("\n=== RAPORT OPÓŹNIEŃ PRZETWARZANIA (LATENCY REPORT) ===")
        print(f"Średnia latencja algorytmiczna (Core): {np.mean(algo_latencies):.3f} ms")
        print(f"Maksymalna latencja algorytmiczna (Worst-case): {np.max(algo_latencies):.3f} ms")
        print(f"Średnia latencja wyświetlania (OpenCV GUI narzut): {np.mean(gui_latencies):.3f} ms")
        print(f"Całkowity średni czas obrotu pętli: {np.mean(algo_latencies) + np.mean(gui_latencies):.3f} ms")
        print("======================================================")

if __name__ == "__main__":
    main()