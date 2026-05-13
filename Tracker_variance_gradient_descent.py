from torchvision.transforms.functional import gaussian_blur
from ShapeLoader_ref import ShapeLoader
import numpy as np
import cv2
import torch
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
# wariancja, 1 czas ref, gradient ascent, bilinear voting
class Tracker:

    def __init__(self, pathEvents, pathCalib, device, batch_size, initialROI, startTime):
        self.path = pathEvents
        self.shape_loader = ShapeLoader(pathEvents, pathCalib, startTime)
        self.shape_loader.load_events()
        self.height, self.width = self.shape_loader.sensor_shape
        self.current_index = 0
        self.device = device
        self.batch_size = batch_size
        self.ROI = initialROI
        self.numberParameters = 2  # vx, vy
        self.maxIterations = 100
        self.blurSigma = 0
        self.saved_first_iwe = False  # flaga kontrolna

    def process_events(self):
        # Start flow
        flow = torch.zeros(self.numberParameters, dtype=torch.float64, device=self.device)
        batch_num = 0
        flow = torch.tensor([1.324673, -2.4512532], dtype=torch.float64, device=self.device)
        while self.current_index < self.shape_loader.num_events:
            batch_num += 1
            start_idx = self.current_index
            end_idx = min(self.current_index + self.batch_size, self.shape_loader.num_events)

            events = torch.tensor(
                self.shape_loader.get_events(start_idx, end_idx),
                dtype=torch.float64, device=self.device
            )
            self.current_index = end_idx

            # Debug: liczba eventów w batchu
            print(f"[DEBUG] Batch {batch_num}: {len(events)} total events")

            # Filtr ROI
            x, y, w, h = self.ROI
            mask = (events[:, 1] >= x) & (events[:, 1] < x + w) & (events[:, 2] >= y) & (events[:, 2] < y + h)
            eventsROI = events[mask]

            # Debug: ile eventów w ROI
            print(f"[DEBUG] Batch {batch_num}: {len(eventsROI)} events in ROI, ROI {x, y}, flow {flow}")
            

            # Pokaż pierwsze 5 eventów w ROI
            if len(eventsROI) > 0:
                print(f"[DEBUG] Sample events in ROI: {eventsROI[:5]}")

            if len(eventsROI) == 0:
                print(f"[DEBUG] Batch {batch_num}: No events in the ROI!")
                continue

            flow = self.optimize(eventsROI, flow, self.device, maxIterations=self.maxIterations)

            costOriginal, _ = self.calculate_cost(
                eventsROI, torch.zeros(self.numberParameters, dtype=torch.float64, device=self.device)
            )
            costWarped, _ = self.calculate_cost(eventsROI, flow)

            print(
                f"[DEBUG] Batch {batch_num}: Cost Original: {costOriginal.item():.6f}, Cost Warped: {costWarped.item():.6f}")

            iweOriginal, _ = self.calculate_iwe_and_iwe_deriv(events)
            warped_events, _ = self.warp_events(events, flow, reference=0.5)
            iweWarpedMiddle, _ = self.calculate_iwe_and_iwe_deriv(warped_events)

            # Update ROI
            x += flow[0].item()
            y += flow[1].item()
            oldROI = self.ROI
            self.ROI = (x, y, w, h)

            # Display IWE
            self.display_iwe_roi(iweOriginal, self.ROI, oldROI)
            self.display_iwe(iweWarpedMiddle)

            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break

    def warp_events(self, events, flow, reference):
        ts = events[:, 0]
        x = events[:, 1]
        y = events[:, 2]

        maxTs = ts[-1]
        minTs = ts[0]
        tSpan = maxTs - minTs
        print(tSpan)
        self.tSpan = tSpan
        tsRef = minTs + tSpan * reference
        dt = (ts - tsRef)/tSpan

        warpedEvents = events.clone()
        warpedEvents[..., 0] = dt
        warpedEvents[..., 1] = x - dt * flow[0]
        warpedEvents[..., 2] = y - dt * flow[1]

        return warpedEvents, dt

    def display_3d_projection(self, events, flow, reference=0.5):
        # 1. Rzutowanie zdarzeń przy użyciu Twojej logiki
        warped_events, _ = self.warp_events(events, flow, reference)

        # 2. Pobranie danych z GPU (PyTorch) do NumPy na CPU
        ts = events[:, 0].detach().cpu().numpy()
        x = events[:, 1].detach().cpu().numpy()
        y = events[:, 2].detach().cpu().numpy()

        x_warped = warped_events[:, 1].detach().cpu().numpy()
        y_warped = warped_events[:, 2].detach().cpu().numpy()

        # Ze względów wydajnościowych (matplotlib w 3D bywa wolny), ograniczamy
        # liczbę punktów na wykresie np. do 500, wybierając je równomiernie.
        max_points = 500
        if len(ts) > max_points:
            indices = np.linspace(0, len(ts) - 1, max_points, dtype=int)
            ts, x, y = ts[indices], x[indices], y[indices]
            x_warped, y_warped = x_warped[indices], y_warped[indices]

        # 3. Tworzenie okna z dwoma wykresami
        fig = plt.figure(figsize=(14, 6))

        # ---- Lewy wykres: Przestrzeń 3D ----
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.set_title("Oryginalne zdarzenia 3D i płaszczyzna rzutu", fontsize=12)

        # Chmura oryginalnych zdarzeń
        ax1.scatter(x, y, ts, c=ts, cmap='viridis', s=10, alpha=0.8, label="Zdarzenia (x,y,t)")

        # Czas płaszczyzny referencyjnej (odpowiednik reference=0.5)
        ts_ref = np.min(ts) + (np.max(ts) - np.min(ts)) * reference

        # Płaszczyzna z rzutami zdarzeń (IWE w 3D)
        ax1.scatter(x_warped, y_warped, np.full_like(ts, ts_ref), color='red', s=5, label='Rzuty IWE')

        # Linie pomocnicze (trajektorie rzutowania) - rysujemy tylko kilkanaście dla przejrzystości
        step = max(1, len(ts) // 15)
        for i in range(0, len(ts), step):
            ax1.plot([x[i], x_warped[i]], [y[i], y_warped[i]], [ts[i], ts_ref], 'k--', alpha=0.4)

        ax1.set_xlabel('X (piksele)')
        ax1.set_ylabel('Y (piksele)')
        ax1.set_zlabel('Czas t')
        ax1.view_init(elev=20, azim=-60)  # Dobry kąt początkowy
        ax1.legend()

        # ---- Prawy wykres: Sam rzut IWE (2D) ----
        ax2 = fig.add_subplot(122)
        ax2.set_title(f"Złożone zdarzenia 2D\n(Flow: vx={flow[0].item():.2f}, vy={flow[1].item():.2f})", fontsize=12)
        ax2.scatter(x_warped, y_warped, c='red', s=2, alpha=0.5)

        # Utrzymanie proporcji z ROI
        ax2.set_xlim(min(x) - 2, max(x) + 2)
        ax2.set_ylim(min(y) - 2, max(y) + 2)
        ax2.invert_yaxis()  # Oś Y w dół tak jak w obrazach OpenCV
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')

        plt.tight_layout()
        plt.show()  # To zablokuje wykonanie kodu, aż zamkniesz okno wykresu
    def calculate_iwe_and_iwe_deriv(self, events, dt = 0): # musze podać dt bo wyrzuci błąd jeśli iwe i iwe_deriv są w jednej funkcji
        image = events.new_zeros((self.height * self.width))
        image_deriv = events.new_zeros((self.height * self.width, 2))
        floor_xy = torch.floor(events[..., 1:3])
        frac_xy = events[..., 1:3] - floor_xy
        floor_xy = floor_xy.long()
        x = floor_xy[..., 0]
        y = floor_xy[..., 1]

        inds = torch.cat(
            [x + y * self.width, x + (y + 1) * self.width, (x + 1) + y * self.width, (x + 1) + (y + 1) * self.width],
            dim=-1)
        inds_mask = torch.cat([
            (0 <= x) * (x < self.width) * (0 <= y) * (y < self.height),
            (0 <= x) * (x < self.width) * (0 <= y + 1) * (y + 1 < self.height),
            (0 <= x + 1) * (x + 1 < self.width) * (0 <= y) * (y < self.height),
            (0 <= x + 1) * (x + 1 < self.width) * (0 <= y + 1) * (y + 1 < self.height)], dim=-1)
        inds = (inds * inds_mask).long()

        w_pos0 = (1 - frac_xy[..., 0]) * (1 - frac_xy[..., 1])
        w_pos1 = (1 - frac_xy[..., 0]) * frac_xy[..., 1]
        w_pos2 = frac_xy[..., 0] * (1 - frac_xy[..., 1])
        w_pos3 = frac_xy[..., 0] * frac_xy[..., 1]
        dx_pos0 = -(1-frac_xy[..., 1])
        dx_pos1 = -frac_xy[..., 1]
        dx_pos2 = 1 - frac_xy[..., 1]
        dx_pos3 = frac_xy[..., 1]

        dy_pos0 = -(1-frac_xy[..., 0])
        dy_pos1 = 1 - frac_xy[..., 0]
        dy_pos2 = -frac_xy[..., 0]
        dy_pos3 = frac_xy[..., 0]

        d_wrt_vx = torch.cat([
            dx_pos0 * dt,
            dx_pos1 * dt,
            dx_pos2 * dt,
            dx_pos3 * dt
        ], dim=-1)

        d_wrt_vy = torch.cat([
            dy_pos0 * dt,
            dy_pos1 * dt,
            dy_pos2 * dt,
            dy_pos3 * dt
        ], dim=-1)
        vals = torch.cat([w_pos0, w_pos1, w_pos2, w_pos3], dim=-1)
        vals = vals * inds_mask
        image.scatter_add_(0, inds, vals)

        image = image.reshape((self.height, self.width))

        image = image[None, ...]
        if self.blurSigma > 0:
            image = gaussian_blur(image, kernel_size=3, sigma=self.blurSigma)

        vals = torch.stack([d_wrt_vx, d_wrt_vy], dim=-1)  # (N,4,2)
        vals = vals * inds_mask[..., None]

        # scatter_add dla obu kanałów
        image_deriv.scatter_add_(0, inds.unsqueeze(-1).expand(-1, 2), vals.view(-1, 2))

        image_deriv = image_deriv.reshape((self.height, self.width, 2))
        image_deriv = image_deriv[None, ...]

        # to psuje działanie skryptu
        if self.blurSigma > 0:
            image_deriv = gaussian_blur(image_deriv, kernel_size=3, sigma=self.blurSigma)

        return torch.squeeze(image), torch.squeeze(image_deriv)

    def variance_loss(self, iwe):
        print("\n--- DEBUG START: variance_loss (TORCH) ---")
        flattened = iwe.view(-1)

        # --- Krok 1: N ---
        n = flattened.numel()
        print(f"[TORCH] Krok 1: N = {n}")

        # --- Krok 2: Total ---
        total = torch.sum(flattened)
        print(f"[TORCH] Krok 2: Total (final) = {total.item():.10f}")

        # --- Krok 3: Mean ---
        mean = total / n
        print(f"[TORCH] Krok 3: Mean = {mean.item():.10f}")

        # --- Krok 4: Squared Diffs Sum ---
        # Dla debugowania, używamy pętli, aby dopasować logi z FXP
        # squared_diffs_sum = torch.tensor(0.0, dtype=torch.float64, device=iwe.device)
        # for i, val in enumerate(flattened):
        #     diff = val - mean
        #     sq = diff * diff
        #     squared_diffs_sum = squared_diffs_sum + sq
        #     # if i < 5:  # Drukuj pierwsze 5 wartości
        #     #     print(
        #     #         f"  [TORCH] SqSum iter {i}: diff={diff.item():.10f}, sq={sq.item():.10f}, sum={squared_diffs_sum.item():.10f}")

        # Oryginalna szybsza wersja:
        squared_diffs_sum = torch.sum((flattened - mean) ** 2)
        print(f"[TORCH] Krok 4: Squared_diff_sum (final) = {squared_diffs_sum.item():.10f}")

        # --- Krok 5: Variance ---
        variance = squared_diffs_sum / n
        print(f"[TORCH] Krok 5: Variance = {variance.item():.10f}")
        print("--- DEBUG KONIEC: variance_loss (TORCH) ---\n")

        return variance

    def variance_deriv(self, iwe, iwe_deriv):
        print("\n--- DEBUG START: variance_deriv (TORCH) ---")
        I = iwe.view(-1)
        dI = iwe_deriv.view(-1, 2)
        N = I.numel()
        print(f"[TORCH] Krok 1: N = {N}")

        I_mean = torch.mean(I)
        print(f"[TORCH] Krok 2: Mean = {I_mean.item():.10f}")

        dI_mean = torch.mean(dI, dim=0)

        print(f"[TORCH] Krok 3: dI Mean = {dI_mean}")
        # ----------------------------------------------------------------------------------

        diff = (I - I_mean).unsqueeze(1)
        print(diff.shape)

        # --- New Debugging Print ---
        print("\n[TORCH] Krok 4: Pierwsze 5 wartości 'diff':")
        # Use 'min(5, N)' in case N is less than 5
        for i in range(min(5, N)):
            print(f"Index {i}: {diff[i].item():.10f}")
        print("------------------------------------------")
        # ---------------------------

        grad = (2.0 / N) * torch.sum(diff * (dI - dI_mean), dim=0)
        print(f"Debug: GRADIENTY = {grad}")
        return grad

    def calculate_cost(self, events, flow):
        iweOriginal, _ = self.calculate_iwe_and_iwe_deriv(events)
        eventsWarpedMiddle, dt = self.warp_events(events, flow, reference=0.5)
        iweMiddle, iweDerivMiddle = self.calculate_iwe_and_iwe_deriv(eventsWarpedMiddle, dt)
        lossWarpedMiddle = self.variance_loss(iweMiddle)
        gradDerivMiddle = self.variance_deriv(iweMiddle, iweDerivMiddle)
        return lossWarpedMiddle, gradDerivMiddle

    def debug_calculate_iwe_and_iwe_deriv_torch(self, events, dt=None):
        print("==== DEBUG: calculate_iwe_and_iwe_deriv_torch ====")
        if dt is None:
            dt = torch.zeros(len(events), dtype=events.dtype, device=events.device)
        print(f"Events count: {len(events)}")
        print(f"dt shape: {dt.shape}")
        print(f"Image size expected: {self.height * self.width}")

        image = torch.zeros(self.height * self.width, dtype=events.dtype, device=events.device)
        image_deriv = torch.zeros(self.height * self.width, 2, dtype=events.dtype, device=events.device)

        floor_xy = torch.floor(events[..., 1:3] + 1e-6)
        frac_xy = events[..., 1:3] - floor_xy
        floor_xy = floor_xy.long()
        x = floor_xy[..., 0]
        y = floor_xy[..., 1]

        print("First 5 floor/frac xy debug:")
        for i in range(min(5, len(events))):
            print(
                f"  Event {i}: x={float(events[i, 1]):.3f}, y={float(events[i, 2]):.3f}, floor_x={x[i].item()}, floor_y={y[i].item()}, frac_x={float(frac_xy[i, 0]):.3f}, frac_y={float(frac_xy[i, 1]):.3f}")

        inds = torch.cat(
            [x + y * self.width, x + (y + 1) * self.width, (x + 1) + y * self.width, (x + 1) + (y + 1) * self.width],
            dim=-1)
        inds_mask = torch.cat([
            (0 <= x) * (x < self.width) * (0 <= y) * (y < self.height),
            (0 <= x) * (x < self.width) * (0 <= y + 1) * (y + 1 < self.height),
            (0 <= x + 1) * (x + 1 < self.width) * (0 <= y) * (y < self.height),
            (0 <= x + 1) * (x + 1 < self.width) * (0 <= y + 1) * (y + 1 < self.height)], dim=-1)
        inds = (inds * inds_mask).long()

        print("First 5 candidate indices debug:")
        for i in range(min(5, len(events))):
            for j in range(4):
                idx = inds[i * 4 + j].item()
                x0_int = x[i].item()
                y0_int = y[i].item()
                in_bounds = inds_mask[i * 4 + j].item() > 0
                print(f"  Event {i} candidate {j}: idx={idx}, x0={x0_int}, y0={y0_int}, in_bounds={in_bounds}")

        w_pos0 = (1 - frac_xy[..., 0]) * (1 - frac_xy[..., 1])
        w_pos1 = (1 - frac_xy[..., 0]) * frac_xy[..., 1]
        w_pos2 = frac_xy[..., 0] * (1 - frac_xy[..., 1])
        w_pos3 = frac_xy[..., 0] * frac_xy[..., 1]

        dx_pos0 = -(1 - frac_xy[..., 1])
        dx_pos1 = -frac_xy[..., 1]
        dx_pos2 = 1 - frac_xy[..., 1]
        dx_pos3 = frac_xy[..., 1]

        dy_pos0 = -(1 - frac_xy[..., 0])
        dy_pos1 = 1 - frac_xy[..., 0]
        dy_pos2 = -frac_xy[..., 0]
        dy_pos3 = frac_xy[..., 0]

        print("First 5 weights debug:")
        for i in range(min(5, len(events))):
            print(
                f"  Event {i}: w0={w_pos0[i].item():.3f}, w1={w_pos1[i].item():.3f}, w2={w_pos2[i].item():.3f}, w3={w_pos3[i].item():.3f}")

        d_wrt_vx = torch.cat([
            dx_pos0 * dt,
            dx_pos1 * dt,
            dx_pos2 * dt,
            dx_pos3 * dt
        ], dim=-1)

        d_wrt_vy = torch.cat([
            dy_pos0 * dt,
            dy_pos1 * dt,
            dy_pos2 * dt,
            dy_pos3 * dt
        ], dim=-1)

        vals = torch.cat([w_pos0, w_pos1, w_pos2, w_pos3], dim=-1)
        vals = vals * inds_mask

        print(f"Length inds: {inds.shape[0]}, length vals: {vals.shape[0]}, length d_wrt_vx: {d_wrt_vx.shape[0]}")

        print("First 10 indices and values:")
        for i in range(min(10, inds.shape[0])):
            print(f"  inds[{i}]={inds[i].item()}, vals[{i}]={vals[i].item():.3f}")

        image.scatter_add_(0, inds, vals)

        for i in range(len(inds)):
            idx = inds[i]
            if idx >= 0:
                image_deriv[idx, 0] += d_wrt_vx[i]
                image_deriv[idx, 1] += d_wrt_vy[i]

        image_2d = image.reshape(self.height, self.width)
        image_deriv_2d = image_deriv.reshape(self.height, self.width, 2)

        print(f"Image 2D shape: {image_2d.shape[0]} rows, {image_2d.shape[1]} cols")
        # Debug print non-zero pixels in image_2d
        nonzero_inds = torch.nonzero(image_2d)
        print("Non-zero pixels in image_2d (y, x) and their values:")
        for idx in nonzero_inds:
            y, x = idx.tolist()
            value = image_2d[y, x].item()
            print(f"  pixel at (y={y}, x={x}): {value:.3f}")
        print("==== END DEBUG ====")
        return image_2d, image_deriv_2d

    def optimize(self, events, flow0, device, maxIterations=100, lr=5):
        flow = flow0.clone().to(device).double()
        flow.requires_grad = False

        for i in range(maxIterations):
            loss, grad = self.calculate_cost(events, flow)
            flow = flow - lr * grad
            # if i % 10 == 0 or i == maxIterations - 1:
            print(f"Iteration {i}, Loss: {loss.item():.6f}, Flow: {flow.detach().cpu().numpy()}")
        return flow
    
    def optimize_torch(self, events, flow0, device, maxIterations=100, lr=5):
        flow = flow0.clone().to(device).double()
        flow.requires_grad = True
        optimizer = torch.optim.Adam([flow], lr=lr)

        for i in range(maxIterations):
            optimizer.zero_grad()
            loss, _ = self.calculate_cost(events, flow)
            loss.backward()
            optimizer.step()
            if i % 10 == 0 or i == maxIterations - 1:
                print(f"Iteration {i}, Loss: {loss.item():.6f}, Flow: {flow.detach().cpu().numpy()}")
        return flow.detach()
    
    def display_iwe(self, image):
        # Display the image
        image_np = image.cpu().detach().numpy()
        image_np = cv2.normalize(image_np, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow("IWE Warped", image_np)
        cv2.waitKey(1)

    def display_iwe_roi(self, image, roi, oldROI):
        # Display the image with a red bounding box around the ROI
        image_np = image.cpu().detach().numpy()
        image_np = cv2.normalize(image_np, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        x, y, w, h = roi
        old_x, old_y, old_w, old_h = oldROI
        imageBGR = cv2.cvtColor(image_np, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(imageBGR, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 1)
        cv2.rectangle(imageBGR, (int(old_x), int(old_y)), (int(old_x + old_w), int(old_y + old_h)), (255, 0, 0), 1)
        cv2.imshow("IWE Original", imageBGR)
        cv2.waitKey(1)

if __name__ == "__main__":
    # Example usage

    cuda_available = torch.cuda.is_available()
    if cuda_available:
        device = "cuda"
    else:
        device = "cpu"
    print(device)

    batch_size = 5000

    # Translation shapes dataset
    pathEvents = r"C:\Users\hp\Documents\inz\model\Shapes_Translation\events.txt"
    pathCalib = r"C:\Users\hp\Documents\inz\model\Shapes_Translation\calib.txt"
    initialROI = (115, 29, 40, 40)  # (x, y, width, height)
    startTime = 0

    # Rotation shapes dataset
    # pathEvents = r"C:\Users\hp\Documents\inz\model\Shapes_Rotation\events.txt"
    # pathCalib = r"C:\Users\hp\Documents\inz\model\Shapes_Rotation\calib.txt"
    # initialROI = (117, 17, 40, 40) # (x, y, width, height)
    # startTime = 7.61

    tracker = Tracker(pathEvents, pathCalib, device, batch_size, initialROI, startTime)
    tracker.process_events()
