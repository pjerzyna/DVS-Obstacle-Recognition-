# DVS Obstacle Recognition: Real-Time Event-Based Optical Avoidance on Edge Devices

[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.com/)
[![Sensor](https://img.shields.io/badge/Sensor-Prophesee%20GenX320-blue.svg)](https://www.prophesee.ai/)
[![Language](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![SDK](https://img.shields.io/badge/Metavision-SDK-green.svg)](https://www.prophesee.ai/metavision-intelligence/)


> **Course Project**: Dynamic Vision Sensors  
> **Authors**: [Paweł Jerzyna](https://github.com/pjerzyna), Piotr Grzyb, Marcin Dworak  
> **Method**: ORB-inspired keypoint pipeline + RANSAC homography, implemented from scratch in Python/NumPy
> **Tech Stack**: Python (Pandas, SQLAlchemy, Faker), MySQL, PowerBI

################## TO DO ############## 
---

## 📌 Executive Summary / TL;DR

This repository delivers a **real-time, bio-inspired optical avoidance system** designed for resource-constrained edge hardware (Raspberry Pi 5) coupled with a neuromorphic event-based camera (**Prophesee GenX320**). 

By eschewing dense Deep Neural Networks (DNNs) and heavy Contrast Maximization (CM) algorithms—which cause severe CPU saturation and stream desynchronization on edge devices—this C++ pipeline mimics the **looming detection and escape reflexes of flying insects**. It extracts dynamic 2D bounding boxes using **vectorized ARM NEON SIMD instructions** and evaluates a kinematic **Time-to-Collision (TTC)** surface expansion metric.

* 🎬 **[Demo Video 1 (Real-time detection)](./docs_performance/media/demonstrator1.mp4)**
* 🎬 **[Demo Video 2 (TTC Alert)](./docs_performance/media/demonstrator2.mp4)**

### Key Benchmark Metrics (Raspberry Pi 5 @ $320 \times 320$ resolution)
* ⚡ **Mean Core Latency**: **~0.13 ms** per 10 ms slice (**<1.3%** of single-core real-time budget).
* 🚀 **Real-Time Processing Margin**: **76×** factor (processed 26.6 s of high-density event data containing 7.14M events in just **350 ms** CPU time).
* 🛡️ **Zero Packet Loss**: **0%** slice deadline overruns; sustained deterministic **100 Hz** output cadence.
* 💻 **Ultra-Low Resource Footprint**: Consumes **< 2%** single-core CPU load, leaving remaining cores free for drone flight controller integration (PX4 / ArduPilot).

---

## 🏗️ System Architecture & Data Pipeline

The pipeline processes asynchronous `EVT3` event streams $(x, y, p, t)$ through six decoupled C++ processing modules managed by a single `DetectionPipeline` engine.

+--------------------------+
| Prophesee GenX320 Sensor | (320x320 Asynchronous EVT3 Stream)
+--------------------------+
|
v
+--------------------------+      +-----------------------+
| Metavision SDK Driver    | ---> | RAW Stream Recording  | (Optional Disk Logging)
+--------------------------+      +-----------------------+
|
v
+--------------------------+      +-----------------------+
| Spatiotemporal Filter    | ---> | Filtered Recording    | (3 ms Rolling Window, 4-Way Neighborhood)
+--------------------------+      +-----------------------+
|
v
+--------------------------+
| Frame Slicer             | (Partitioning events into DT = 10 ms slices)
+--------------------------+
|
v
+--------------------------+
| Adaptive Calibration     | (Dynamic background activity estimation over first 30 slices)
+--------------------------+
|
v
+--------------------------+
| Geometric Tracker (SIMD) | (Horizontal sectoring: Left/Central/Right, Bounding Box & Centroid)
+--------------------------+
|
v
+--------------------------+
| Kinematic TTC Estimator  | (60 ms sliding window area growth rate A_dot -> TTC calculation)
+--------------------------+
|
v
+--------------------------+      +-----------------------+
| Collision Decision Engine| ---> | Live OpenCV Replay    | (Validation mode: replay_viewer)
| Terminal Telemetry / Alert|      +-----------------------+
+--------------------------+


---

## 📐 Mathematical Foundations

### 1. Spatiotemporal Neighborhood Filtering
To eliminate uncorrelated background thermal noise and hot pixels, each incoming event $e_i = (x_i, y_i, p_i, t_i)$ is evaluated against a 4-connected spatial neighborhood $\mathcal{N}(x_i, y_i)$ within a rolling 3 ms temporal deque:

$$\mathcal{N}(x_i, y_i) = \{(x_i \pm 1, y_i), (x_i, y_i \pm 1)\}$$

An event survives filtering if and only if:

$$\sum_{k \in \mathcal{N}(x_i, y_i)} \mathbb{I}(k) \ge N_{\text{min}} \quad (N_{\text{min}} = 1)$$

### 2. Vectorized Geometric Feature Tracking
Surviving events are mapped across three spatial sectors: **Left** ($x \le 106$), **Central** ($107 \le x \le 213$), and **Right** ($x \ge 214$). For the sector with maximum event count, spatial bounds ($x_{\text{min}}, x_{\text{max}}, y_{\text{min}}, y_{\text{max}}$) and arithmetic centroid $c(t)$ are calculated in a single pass parallelized via **ARM NEON SIMD**:

$$c(t) = \left( \frac{1}{n}\sum_{i=1}^{n} x_i, \; \frac{1}{n}\sum_{i=1}^{n} y_i \right), \quad \Delta c(t) = c(t) - c(t - 1)$$

### 3. Kinematic Time-to-Collision (TTC) Estimation
Rather than recovering explicit 3D metric depth, the system measures the optical footprint expansion rate ($\dot{A}_s = \Delta A_s / \Delta t$) of the 2D bounding box area $A_s(t) = w_s(t) \cdot h_s(t)$ over a 60 ms sliding window:

$$TTC_s(t) = \frac{A_s(t)}{\dot{A}_s} \quad \text{[seconds]}$$

A safety-critical collision alert is triggered whenever:

$$TTC_s(t) < \tau_{\text{danger}} \quad (\tau_{\text{danger}} = 0.45\text{ s})$$

---

## 📊 Performance Benchmarks & Comparison

### Detailed Per-Stage Execution Latency (Raspberry Pi 5)
*Benchmark Dataset: 7.14M events, $320 \times 320$ resolution, 26.6 seconds real-world recording.*

| Pipeline Stage | Mean Latency | p99 Latency | Max Latency | Operational Scaling |
| :--- | :---: | :---: | :---: | :--- |
| **Neighborhood Filter** *(per packet)* | **11.7 µs** | 34.3 µs | 1618 µs | $O(N + W \cdot H)$ (map rebuild) |
| **Geometric Tracker** *(per 10ms slice)* | **14.2 µs** | 97.1 µs | 838 µs | $O(N)$ ($\sim 9\text{ ns/event}$ SIMD) |
| **Kinematic TTC Estimator** *(per slice)*| **0.9 µs** | 20.1 µs | 91 µs | $O(1)$ ($\le 7$ history samples) |
| **Tracker + TTC Total** *(per slice)* | **15.1 µs** | **104.5 µs** | **839 µs** | **Mean core cost: 0.13 ms / slice** |

### Benchmark vs. Standard State-of-the-Art Solutions

| Metric / Framework | Contrast Maximization (CM) | Deep Neural Network (ResNet34) | **Our Bio-Inspired Pipeline** |
| :--- | :---: | :---: | :---: |
| **CPU Core Load** | 100% (CPU Saturation) | 100% (High Thermal Throttling) | **< 2% Single Core Load** |
| **Packet Processing Latency** | > 110 ms | > 150 ms | **0.13 ms (Mean)** |
| **Stream Stability** | ❌ Desynchronization / Errors | ❌ Heavy Latency Bottlenecks | **`100% Deterministic (0% Loss)`** |
| **Real-Time Factor** | < 0.1× (Unusable) | Non-real-time without NPU/GPU | **`76× Real-Time Margin`** |

---

## 🛠️ Hardware & Prerequisites

### Hardware Requirements
* **Sensor**: Prophesee GenX320 Neuromorphic Event Camera ($320 \times 320$ sensor array).
* **Processing Board**: Raspberry Pi 5 (Broadcom BCM2712 Quad-core ARM Cortex-A76 @ 2.4 GHz, 8GB/16GB RAM).

### Software Dependencies
* **Operating System**: Raspberry Pi OS (64-bit / Debian Bookworm).
* **Compiler**: GCC / G++ (C++17 support required).
* **Libraries**:
  * [Prophesee Metavision SDK](https://www.prophesee.ai/metavision-intelligence/) (`metavision-sdk-base`, `core`, `stream`)
  * OpenCV 4.x (Required for offline visualization companion tool `replay_viewer`)
  * CMake $\ge 3.16$

---

## 💻 Build & Installation

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/pjerzyna/DVS-Obstacle-Recognition-.git](https://github.com/pjerzyna/DVS-Obstacle-Recognition-.git)
   cd DVS-Obstacle-Recognition-



## Build on rpi
1. Configure & Compile:

Bash

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4

(Note: The CMakeLists.txt automatically passes -O3 -mcpu=cortex-a76 flags to enable compiler auto-vectorization for ARM NEON SIMD lanes.)

# 🚀 Running the Executables

1. Live Operation on Raspberry Pi 5 (optical_avoidance)

To run the live headless engine directly connected to the GenX320 sensor via USB:

Bash

$ ./optical_avoidance

Terminal Output Example:
Plaintext (yes, it is in polish language)

[DETEKCJA] Centroid: (267, 159) BB: [105x175] Kierunek: W LEWO Sektor: CENTRALNY Punkty: 104
[A KOLIZJA] KOLIZJA CENTRALNY TTC=0.32s | Centroid: (267, 159) Sektor: CENTRALNY


2. Offline Replay & OpenCV Visualization (replay_viewer)

To replay a previously recorded .evt3 event file with bounding box overlays, sector grid lines, and visual TTC alerts:
Bash

./replay_viewer /path/to/recording.evt3



## 📁 Repository Structure
   
   .
   ├── CMakeLists.txt          # Build configuration with ARM NEON optimizations
   ├── include/                # Header files (DetectionPipeline, Filter, Tracker, TTC)
   ├── src/                    # Implementation modules (.cpp)
   │   ├── optical_avoidance.cpp # Main executable for live sensor engine
   │   └── replay_viewer.cpp     # Offline OpenCV visualization companion tool
   ├── docs/                   # Academic paper, diagrams, presentation slides
   ├── tools/                  # Auxiliary scripts & configuration files
   └── README.md               # Project documentation

🔮 Future Work & Roadmap

    Inertial Sensor Fusion (IMU): Integrate continuous-time gyro/accelerometer data to cancel out the drone's own ego-motion and prevent false-positive expansion alerts during banking/pitch maneuvers.

    Multi-Obstacle Connected-Component Clustering: Replace the single bounding box abstraction with multi-cluster tracking to handle complex environments containing multiple moving hazards.

    Closed-Loop Flight Integration: Connect the C++ decision engine directly to a PX4 / ArduPilot flight controller via MAVLink to translate TTC warnings into active evasive thrust vectors.


🙏 Acknowledggments

Special thanks to the Embedded Vision Systems Group at the AGH University of Krakow for providing hardware access, testing facilities, and research guidance throughout this project.