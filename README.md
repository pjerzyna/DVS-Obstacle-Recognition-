# Event-Based Perception for High-Speed Drone Maneuvers on Raspberry Pi

### 🚀 Cel projektu
Celem projektu jest opracowanie i implementacja ultra-wydajnego potoku przetwarzania danych z kamery zdarzeniowej **Prophesee GenX320 (Metavision)** na platformie wbudowanej **Raspberry Pi 5**. 

Projekt skupia się na utrzymaniu precyzyjnej percepcji środowiska podczas gwałtownych manewrów drona (gdzie tradycyjne kamery klatkowe całkowicie zawodzą z powodu rozmycia obrazu). W warstwie algorytmicznej dążymy do optymalizacji metod opartych o **Contrast Maximization (IWE)** pod kątem ograniczeń sprzętowych systemów embedded.

---

### 🧠 Wyzwanie: Hardware vs. High-Speed Data
Głównym problemem badawczym jest ograniczona moc obliczeniowa i przepustowość zapisu na karcie SD urządzenia wbudowanego w starciu z potężnym strumieniem danych zdarzeniowych.

#### Analiza i dobór zbiorów danych:
* **Dataset M3ED (Odrzucony):** Wykorzystuje wysoką rozdzielczość (1280x720) i wyłączony kontroler ERC, co generuje do **200 milionów zdarzeń na sekundę (MEPS)** – wartość nie do przetworzenia w czasie rzeczywistym na SBC.
* **Dataset DAVIS 240C (Odrzucony):** Niższa rozdzielczość (240x180) pozwala na stabilne testowanie algorytmów bez dławienia wątku procesora.
* **Dataset indoor_forward_5_davis_with_gt (Odrzucony):** Zawiera realne dane zebrane z kamery zamontowanej na dronie, co idealnie odwzorowuje docelowe warunki pracy systemu.
* **Postać z kubkiem przed kamerą (Nagrany prze nas):** Zawiera dane nagrane przez nas.

---

## 🛠️ Architektura potoku danych (Pipeline)

Wykorzystaliśmy autorską architekturę zapisu przefiltrowanych struktur do formatu `.raw` - dwuetapowy, zoptymalizowany potok danych. Niemniej przyszły format docelowo piwonien zapisywać się jak prophesee docelowo zapisuje, czyli w fromacie `.raw`:

```
[Kamera GenX320] ──> (Raspberry Pi 5: Skrypt SSH) ──> [Filtr Sąsiedztwa 2D] 
                                                               │
                                                               ▼
                                                       (Zrzut pamięci RAM)
                                                               │
                                                               ▼
[Komputer PC: Wizualizacja] <── (Transfer SCP) <── [wykryty_ruch.bin]
```

1.  **Gromadzenie i filtracja (Raspberry Pi 5):** Strumień na żywo z kamery jest filtrowany sprzętowo (biases) oraz programowo (lekki 4-kierunkowy filtr sąsiedztwa). Aby nie obciążać procesora pakowaniem bitowym, przefiltrowane zdarzenia są zrzucane jako surowy dump pamięci RAM (`.tobytes()`) bezpośrednio do pliku **`.bin`**.
2.  **Analiza i odtwarzanie (PC):** Plik binarny jest przesyłany na komputer za pomocą `scp`. Odczyt danych realizowany jest z pominięciem ciężkiego iteratora Metavision za pomocą `numpy.fromfile()` przy użyciu precyzyjnego wyrównania pamięci do 16 bajtów per zdarzenie (`itemsize: 16`), co odwzorowuje strukturę `EventCD`.  

---

## 📦 # Instrukcja i Wymagania

Aby uruchomić skrypty przetwarzające i zintegrować je z ekosystemem Prophesee, wymagane jest przygotowanie odpowiedniego środowiska Python.

### Wymagane biblioteki

```bash
pip install numpy matplotlib h5py
```


### Środowisko Metavision Intelligence
Skrypty wymagają zainstalowanego systemu **Metavision SDK** (OpenEB) wraz z powiązanymi bindingami dla języka Python (`metavision_core`, `metavision_hal`). Instrukcja jak zadziałało to u mnie, jest w pliku `info.ipynb`

---

## 📊 Demonstracja i Zasoby

Badania naukowe potwierdzają, że kamery zdarzeniowe potrafią śledzić ruch z mikrosekundową latencją. Ten projekt udowadnia, że zaawansowaną filtrację i tracking obiektów można z powodzeniem realizować na mobilnej i relatywnie taniej platformie, jaką jest Raspberry Pi 5.

### 🔗 Zasoby projektu

| Zasób | Opis | Link |
| :--- | :--- | :--- |
| **DVS Documentation** | Dokumentacja postępu prac oraz analizy matematycznej | [Otwórz folder Google Drive](https://docs.google.com/document/d/1JVSYr9W3rMijzj9mugauywRmXaeJ3GHVQlDzZBNm6CM/edit?usp=sharing) |
| **Materiały źródłowe** | Logi, próbki nagrań oraz zebrane pliki binarne | [Zobacz pliki na Drive](https://drive.google.com/drive/folders/1pNTtwfE_gR4jPu4br1rLfwvJslzubs4B?dmr=1&ec=wgc-drive-module-goto) |