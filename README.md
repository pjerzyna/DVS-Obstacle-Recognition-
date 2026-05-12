# Event-Based Perception for High-Speed Drone Maneuvers on Raspberry Pi

### 🚀 Cel projektu
Celem projektu jest opracowanie i implementacja wydajnego potoku przetwarzania danych z kamer zdarzeniowych na platformie **Raspberry Pi**. Projekt skupia się na utrzymaniu precyzyjnej percepcji podczas **gwałtownych manewrów drona**, gdzie tradycyjne systemy wizyjne zawodzą z powodu rozmycia obrazu.

### 🧠 Wyzwanie: Hardware vs. High-Speed Data
Głównym problemem badawczym jest ograniczona przepustowość urządzenia wbudowanego w starciu z surowym strumieniem danych.
*   **Dataset M3ED:** Wykorzystuje wysoką rozdzielczość (1280x720) i wyłączony kontroler ERC, co generuje do **200 milionów zdarzeń na sekundę (MEPS)**.
*   **Ograniczenia RPi:** Konieczność drastycznej optymalizacji, odszumiania oraz stosowania asynchronicznych reprezentacji danych, aby osiągnąć działanie w czasie rzeczywistym.

---

### 🛠 Plan działania i Architektura systemu

#### 1. Preprocessing Danych (M3ED Dataset)
Analiza sekwencji *UAV Indoor* z datasetu M3ED.

#### 2. Reprezentacja Zdarzeń (Event Representation)

#### 3. Zadanie główne: Kompensacja Ruchu i Fuzja Sensoryczna
tj. Wykorzystanie danych z żyroskopu do odróżnienia ruchu własnego drona (*ego-motion*) od obiektów poruszających się w otoczeniu.

#### 4. Optymalizacja Hardware-Oriented

#### 5. Test na PC

#### 6. Test na RaspberryPi

---

## 📊 Demonstracja

Badania (np. Muegglera) potwierdzają, że kamery zdarzeniowe potrafią śledzić ruch z mikrosekundową latencją. Ten projekt udowadnia, że jest to możliwe do zrealizowania na (relatywnie) **taniej, mobilnej platformie obliczeniowej**.


### 🔗 Zasoby projektu

| Zasób | Opis | Link |
| :--- | :--- | :--- |
| **DVS** | Dokumentacja postępu prac (Google Drive) | [Otwórz folder](https://docs.google.com/document/d/1JVSYr9W3rMijzj9mugauywRmXaeJ3GHVQlDzZBNm6CM/edit?usp=sharing) |
| **Źródła** | Materiały źródłowe | [Zobacz pliki](https://drive.google.com/drive/folders/1pNTtwfE_gR4jPu4br1rLfwvJslzubs4B?dmr=1&ec=wgc-drive-module-goto) |