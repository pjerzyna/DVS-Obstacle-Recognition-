import cv2
import torch

class ShapeLoader:
    def __init__(self, pathEvents, pathCalib, startTime, device="cpu"):
        self.pathEvents = pathEvents
        self.pathCalib = pathCalib
        self.startTime = startTime
        self.device = device

    def load_events(self):
        self.sensor_shape = (180, 240)
        events = []

        with open(self.pathEvents, 'r') as file:
            for line in file:
                if line.startswith("#") or line.strip() == "":
                    continue
                parts = line.strip().split()
                if len(parts) != 4:
                    continue
                t, x, y, p = map(float, parts)
                if self.startTime is None or t >= self.startTime:
                    events.append([1e6*t, x, y, p])

        # wrzucamy od razu w tensor
        self.events = torch.tensor(events, dtype=torch.float32, device=self.device)
        self.num_events = self.events.shape[0]

    def rectify_events(self):
        import numpy as np

        # wczytanie parametrów kalibracyjnych
        with open(self.pathCalib, "r") as f:
            line = f.readline().strip()
            calib = list(map(float, line.split()))
            if len(calib) != 9:
                raise ValueError(
                    f"Niepoprawna liczba parametrów w {self.pathCalib} (oczekiwano 9, jest {len(calib)})."
                )

        fx, fy, cx, cy, k1, k2, p1, p2, k3 = calib

        cameraMatrix = np.array([[fx, 0, cx],
                                 [0, fy, cy],
                                 [0, 0, 1]], dtype=np.float32)
        distCoeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        # bierzemy tylko kolumny x,y → (N,2)
        pts_np = self.events[:, 1:3].cpu().numpy().reshape(-1, 1, 2)

        # korekcja OpenCV
        undistorted = cv2.undistortPoints(pts_np, cameraMatrix, distCoeffs, P=cameraMatrix)
        undistorted = undistorted.reshape(-1, 2)  # (N,2)

        # konwersja z powrotem do torch
        undistorted_torch = torch.from_numpy(undistorted).to(self.device)

        # podmieniamy x,y w tensorze events
        rectified = self.events.clone()
        rectified[:, 1:3] = undistorted_torch

        # maska: zostawiamy tylko eventy w sensorze
        mask_x = (rectified[:, 1] >= 0) & (rectified[:, 1] < self.sensor_shape[1])
        mask_y = (rectified[:, 2] >= 0) & (rectified[:, 2] < self.sensor_shape[0])
        mask = mask_x & mask_y

        self.rectified = rectified[mask]
        self.num_rectified_events = self.rectified.shape[0]

    def get_events(self, idx_start, idx_end):
        return self.events[idx_start:idx_end]

    def get_rectified(self, idx_start, idx_end):
        return self.rectified[idx_start:idx_end]
