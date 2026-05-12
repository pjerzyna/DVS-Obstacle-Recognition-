# Event-Based Perception for High-Speed Drone Maneuvers on Raspberry Pi

### 🚀 Cel projektu
Celem projektu jest opracowanie i implementacja wydajnego potoku przetwarzania danych z kamery zdarzeniowej Prophesee’s GenX320 Metavision na platformie Raspberry Pi 5. Projekt skupia się na utrzymaniu precyzyjnej percepcji podczas imitacji gwałtownych manewrów drona, gdzie tradycyjne systemy wizyjne zawodzą z powodu rozmycia obrazu. Chcemy podejść do tego projektu od storny tematu Contrast Maximization.
  

### 🧠 Wyzwanie: Hardware vs. High-Speed Data
Głównym problemem badawczym jest ograniczona przepustowość urządzenia wbudowanego w starciu z surowym strumieniem danych.
*   **Dataset M3ED [NIE]:** Wykorzystuje wysoką rozdzielczość (1280x720) i wyłączony kontroler ERC, co generuje do **200 milionów zdarzeń na sekundę (MEPS)**.
*   **Dataset DAVIS 240C [TAK]:** Niższa rozdzielczość zdjęć (240x180) przemówiła na korzyść w wykorzystaniu tego zbioru danych
*   **Ograniczenia RPi:** Konieczność drastycznej optymalizacji, aby urządzenie osiągnęło jak najlepszy wynik

---


## 📊 Demonstracja

Badania (np. Muegglera) potwierdzają, że kamery zdarzeniowe potrafią śledzić ruch z mikrosekundową latencją. Ten projekt udowadnia, że jest to możliwe do zrealizowania na relatywnie taniej, mobilnej platformie obliczeniowej tj. Raspberry Pi.


### 🔗 Zasoby projektu

| Zasób | Opis | Link |
| :--- | :--- | :--- |
| **DVS** | Dokumentacja postępu prac (Google Drive) | [Otwórz folder](https://docs.google.com/document/d/1JVSYr9W3rMijzj9mugauywRmXaeJ3GHVQlDzZBNm6CM/edit?usp=sharing) |
| **Źródła** | Materiały źródłowe | [Zobacz pliki](https://drive.google.com/drive/folders/1pNTtwfE_gR4jPu4br1rLfwvJslzubs4B?dmr=1&ec=wgc-drive-module-goto) |