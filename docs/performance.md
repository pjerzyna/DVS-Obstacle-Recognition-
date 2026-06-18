# Wydajność i złożoność — optical_avoidance

Dokument opisuje (1) metryki wydajności wbudowane w pipeline detekcji oraz
(2) analizę złożoności obliczeniowej i parametrycznej. Instrumentacja jest
opcjonalna i **zerowo-narzutowa, gdy wyłączona** (bramkowana wskaźnikiem
`PerfMetrics*` w `DetectionPipeline`).

Pipeline (wspólny dla `main` live i `replay_viewer`):

```
NeighborhoodFilter ──(per paczka SDK)──▶ FrameSlicer(10 ms)
        │                                      │
        ▼                                      ▼
   filtr czas.-przestrz.              ObstacleTracker ──▶ TtcEstimator   (per slice)
```

---

## 1. Metryki wydajności

### 1.1 Jak włączyć

Wszystkie trzy binarki przyjmują te same flagi:

```bash
# Headless benchmark (najszybszy, do powtarzalnych pomiarów; bez GUI)
./build/benchmark /home/sterydy/output/wykryty_ruch.raw \
    --metrics output/metrics_benchmark.csv \
    --metrics-json output/metrics_benchmark_summary.json

# Replay z podglądem + metryki
./build/replay_viewer /home/sterydy/output/wykryty_ruch.raw --metrics

# Tryb live (kamera) + metryki
./build/optical_avoidance --metrics
```

- Bez flagi `--metrics`/`--metrics-json` zachowanie jest **identyczne jak dotąd**.
- Gdy ścieżka jest pominięta, pliki trafiają do katalogu `output/`
  (`resolve_output_dir`, ten sam mechanizm co nagrania `.raw`).
- `benchmark` domyślnie odtwarza plik „tak szybko jak CPU pozwala”
  (`FileConfigHints::real_time_playback(false)`); flaga `--realtime` przywraca
  tempo nagrania.

### 1.2 Definicje metryk

Instrumentacja (`include/perf_metrics.hpp`) mierzy `std::chrono::steady_clock`
wokół każdego etapu i zapisuje dwa poziomy danych:

**Per paczka SDK** (`*_batches.csv`): `raw_events`, `filtered_events`, `filter_us`.

**Per slice (10 ms)** (główny CSV): `slice_events`, `tracker_us`, `ttc_us`,
`slice_total_us` (=tracker+ttc), `filter_buffer` (rozmiar `temporal_buffer_`),
`ttc_history` (najw. okno `history_`), `valid`, `danger`, `sector`, `ttc_value`
(TTC sektora dominującego), `expanding`, `growth_rate`, `relative_growth`.

| Metryka | Definicja |
|---|---|
| **Throughput** | `total_raw_events / suma_czasów_etapów` [Mev/s] |
| **Real-time factor** | `czas_zdarzeń_pokryty / czas_przetwarzania` (×) |
| **Latencja per-etap** | mean / median / p95 / p99 / max [µs] dla filter, tracker, ttc, tracker+ttc |
| **Budget overrun** | % slice'ów, w których tracker+ttc > 10 ms |
| **Peak buffer** | szczytowy rozmiar `temporal_buffer_` (filtr) i `history_` (TTC) |
| **Skuteczność filtra** | `1 − filtered/raw` [%] (odrzucone zdarzenia) |
| **Detekcje** | liczba i % slice'ów z `valid`, liczba epizodów i ich śr. czas |
| **Kolizje (danger)** | liczba i % slice'ów z `danger`, liczba epizodów |
| **Detection latency** | czas do pierwszego `valid` slice w nagraniu [s] |
| **Jitter** | liczba przełączeń flagi `valid` na sekundę (stabilność) |

Statystyki zapisywane są też zbiorczo w `*_summary.json`.

### 1.3 Wyniki empiryczne

Sprzęt: Raspberry Pi 5 (Cortex-A76), build `Release -O3 -mcpu=cortex-a76`.
Nagranie: `wykryty_ruch.raw` (320×320, 7,14 mln zdarzeń, ~26,6 s zdarzeń).

```
Paczki SDK:           26 386
Slice'y (10 ms):      2 663
Zdarzenia surowe:     7 143 802
Zdarzenia po filtrze: 5 171 847   (odrzucono 27,6%)
Throughput:           20,4 Mev/s
Real-time factor:     76×           (350 ms CPU na 26,6 s zdarzeń)
Budget overrun 10ms:  0,00%
Peak bufor filtra:    9 830 zdarzeń
Peak historia TTC:    7 próbek
```

| Etap | mean | median | p95 | p99 | max | [µs] |
|---|---|---|---|---|---|---|
| filtr (paczka) | 11,7 | 9,3 | 21,5 | 34,3 | 1618 | |
| tracker (slice) | 14,2 | 11,0 | 40,0 | 97,1 | 838 | |
| ttc (slice) | 0,9 | 0,1 | 3,0 | 20,1 | 91,0 | |
| tracker+ttc | 15,1 | 11,3 | 43,0 | 104,5 | 839 | |

Jakość (na tym nagraniu): 2496 detekcji (93,7% slice'ów), 375 slice'ów danger
(14,1%), 19 epizodów detekcji (śr. 1,3 s), 160 epizodów kolizji, jitter 1,4/s.

Wykresy: `output/plots/latency_vs_events.png`, `latency_hist.png`,
`detections_timeline.png`, `ttc_threshold_sweep.png`.

---

## 2. Złożoność obliczeniowa

Oznaczenia: `N` — zdarzenia w slice, `B` — zdarzenia w paczce SDK,
`W×H` — wymiary sensora (320×320 = 102 400), `Buf` — rozmiar bufora czasowego
filtra, `Hwin = GROWTH_WINDOW_US / SLICE_DURATION_US` — liczba próbek w oknie TTC
(≈ 6).

| Etap | Operacja dominująca | Złożoność | Uwagi |
|---|---|---|---|
| `NeighborhoodFilter::filter` | `build_map()` → `std::fill` po całej mapie | **O(B + W·H + Buf)** | `W·H` dominuje przy małych paczkach (śr. 270 zdarzeń); patrz niżej |
| → `purge_old` | `pop_front` z deque | O(usuniętych) amort. | |
| → pętla sąsiedztwa | 4 lookupy/zdarzenie | O(B) | O(1) na zdarzenie |
| `FrameSlicer::push` | `push_back` + emisja | O(B) amort. | |
| `FrameSlicer::pop_ready` | `erase(begin())` na `vector` | **O(R)** na pop | R = gotowe slice'y; zwykle 1, ale O(R²) przy zaległościach |
| `ObstacleTracker::process` | jeden przebieg po zdarzeniach | **O(N)** | bbox/centroid/sektory w jednym przejściu |
| `TtcEstimator::estimate` | okno przesuwne na deque | **O(Hwin) ≈ O(1)** | Hwin ograniczone (peak 7) |

### 2.1 Potwierdzenie empiryczne

- **Tracker = O(N) liniowo.** Dopasowanie liniowe: `≈ 9,06 ns/zdarzenie`
  (`latency_vs_events.png`). Współczynnik stały bliski 0 — brak narzutu poza
  przejściem po zdarzeniach.
- **TTC ≈ O(1).** Mediana 0,1 µs, `peak_ttc_history = 7` — niezależne od `N`.
- **Filtr zdominowany przez `W·H`, nie przez zdarzenia.** Średnia paczka to ~270
  zdarzeń, a `filter_us` ma medianę 9,3 µs — co odpowiada `std::fill` ~100 kB
  mapy (memset), a nie liczbie zdarzeń. To główne **wąskie gardło stałokosztowe**:
  `26 386 paczek × 102 400 komórek ≈ 2,7 mld` operacji zerowania mapy.

### 2.2 Zidentyfikowane wąskie gardła i kierunki optymalizacji

1. **`build_map()` czyści całą mapę co paczkę (O(W·H)).** Ponieważ paczki SDK są
   małe, zerowanie mapy dominuje koszt filtra. Możliwe usprawnienia (bez zmiany
   wyników): mapa „znacznikowa” z timestampem zamiast licznika (brak czyszczenia),
   albo czyszczenie tylko komórek dotkniętych w poprzednim oknie.
2. **`FrameSlicer::pop_ready` używa `erase(begin())` na `std::vector` (O(R)).**
   Przy normalnej pracy R≈1, ale przy nadrobieniu zaległości robi się O(R²).
   Tańsza byłaby `std::deque` lub indeks odczytu zamiast `erase`.
3. **Ogon latencji (max filtr 1,6 ms, tracker 0,84 ms)** pochodzi z rzadkich
   dużych paczek/slice'ów (do 26 tys. zdarzeń). p99 nadal < 105 µs ≪ 10 ms — brak
   przekroczeń budżetu real-time.

Aktualny zapas jest duży (real-time factor 76×, 0% przekroczeń budżetu), więc
powyższe to optymalizacje „na zapas”/pod większe sceny, nie konieczność.

---

## 3. Złożoność parametryczna

### 3.1 Inwentaryzacja parametrów

Wszystkie parametry poniżej są **nadpisywalne przy kompilacji** (makra `OA_*`)
bez zmiany wartości domyślnych — co umożliwia przemiatanie (`tools/param_sweep.py
sweep`). Domyślne zachowanie pozostaje niezmienione.

| Parametr | Plik | Domyślnie | Zakres sensowny | Znaczenie / zależności |
|---|---|---|---|---|
| `OA_FILTER_MIN_NEIGHBORS` | `neighborhood_filter.hpp` | 1 | 1–4 | ↑ = silniejszy odrzut szumu, mniej detekcji, szybszy tracker |
| `OA_TEMPORAL_WINDOW_US` | `sensor_config.hpp` | 3000 | 1000–8000 | okno sąsiedztwa czasowego; ↑ = większy `Buf`, więcej kontekstu |
| `OA_SLICE_DURATION_US` | `sensor_config.hpp` | 10000 | 5000–20000 | okno trackera/TTC = budżet real-time; wpływa na `Hwin` |
| `DETECTION_THRESH` / `SECTOR_MIN_EVENTS` | `obstacle_tracker.hpp` | 45 / 15 | 20–200 / 10–80 | progi uznania slice za detekcję; strojone adaptacyjnie |
| `MIN_DETECTION_THRESH` / `MIN_SECTOR_MIN_EVENTS` | `sensor_config.hpp` | 35 / 10 | — | dolne ograniczenie progów adaptacyjnych |
| `SLICE_EVENT_RATIO_PCT` / `SECTOR_EVENT_RATIO_PCT` | `sensor_config.hpp` | 12 / 8 | 5–25 | progi względne (% zdarzeń) — łagodzą próg przy słabym sygnale |
| `OA_TTC_GROWTH_WINDOW_US` | `ttc_estimator.hpp` | 60000 | 30000–120000 | okno wzrostu netto bbox; definiuje `Hwin` |
| `OA_TTC_MIN_GROWTH_RATE` | `ttc_estimator.hpp` | 1500 px²/s | 500–4000 | min. tempo wzrostu pola → odrzuca wiatrak |
| `OA_TTC_MIN_RELATIVE_GROWTH` | `ttc_estimator.hpp` | 0.25 | 0.1–0.5 | min. względny przyrost w oknie → odrzuca oscylacje |
| `OA_TTC_DANGER_THRESHOLD_S` | `ttc_estimator.hpp` | 0.45 s | 0.2–1.0 | próg alarmu kolizji; ↓ = mniej fałszywych alarmów, później ostrzega |

Zależności kluczowe:
- `OA_TTC_GROWTH_WINDOW_US / OA_SLICE_DURATION_US = Hwin` — liczba próbek w oknie
  TTC. Skraca/wydłuża „pamięć” estymatora.
- `MIN_GROWTH_RATE` + `MIN_RELATIVE_GROWTH` razem decydują o odrzuceniu obiektów
  okresowych (wiatrak): muszą być spełnione jednocześnie, by `expanding=true`.
- `MIN_NEIGHBORS` ↔ `TEMPORAL_WINDOW_US`: większe okno zwiększa liczbę sąsiadów,
  więc te same `MIN_NEIGHBORS` stają się łagodniejsze.

### 3.2 Przemiatanie parametrów

**Offline (bez rekompilacji)** — próg TTC re-thresholdowany z zapisanych
`ttc_value`/`expanding`:

```bash
python tools/param_sweep.py ttc-sweep output/metrics_benchmark.csv \
       [--labels tools/labels.csv]
```

Z opcjonalnym plikiem etykiet (`start_us,end_us` realnych kolizji) generuje
krzywe **PR i ROC** po `TTC_DANGER_THRESHOLD_S`. Bez etykiet — wykres liczby
kolizji vs próg.

**Z rekompilacją** — dowolny parametr makro:

```bash
python tools/param_sweep.py sweep --param OA_FILTER_MIN_NEIGHBORS \
       --values 1,2,3 --raw /home/sterydy/output/wykryty_ruch.raw
```

Przykładowy wynik (`MIN_NEIGHBORS`, to samo nagranie):

| MIN_NEIGHBORS | odrzut filtra | tracker mean | throughput | detekcje | kolizje |
|---|---|---|---|---|---|
| 1 | 27,6% | 18,1 µs | 13,7 Mev/s | 2496 | 375 |
| 2 | 40,3% | 12,0 µs | 16,4 Mev/s | 2402 | 412 |
| 3 | 51,4% | 9,0 µs | 17,7 Mev/s | 2233 | 387 |

Widać klasyczny trade-off: agresywniejszy filtr (↑MIN_NEIGHBORS) przyspiesza
tracker i podnosi throughput kosztem liczby detekcji. (Throughput w sweepie jest
niższy niż w pojedynczym czystym przebiegu z powodu równoległej kompilacji.)

---

## 4. Wnioski

- **Real-time z dużym zapasem:** 76× szybciej niż czas zdarzeń, p99 latencji
  tracker+ttc ≈ 105 µs ≪ budżet 10 ms, 0% przekroczeń.
- **Skalowanie potwierdzone:** tracker O(N) (~9 ns/zdarzenie), TTC O(1),
  filtr zdominowany stałym kosztem O(W·H) zerowania mapy.
- **Główne wąskie gardło:** `build_map()` (zerowanie całej mapy co paczkę) —
  kandydat nr 1 do optymalizacji przy większych sensorach / większym ruchu.
- **Narzędzia parametryczne** pozwalają ilościowo dobierać czułość vs koszt oraz
  próg alarmu (PR/ROC), bez ręcznego modyfikowania kodu.
