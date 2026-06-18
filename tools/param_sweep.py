#!/usr/bin/env python3
"""Analiza wydajności i parametryczna dla optical_avoidance.

Trzy tryby (subkomendy):

  analyze    Wykresy z pojedynczego przebiegu (per-slice CSV z benchmark/replay):
             latencja etapów vs liczba zdarzeń, rozkłady latencji, detekcje.

  ttc-sweep  Offline'owe przemiatanie progu TTC_DANGER_THRESHOLD_S na podstawie
             zapisanych ttc_value/expanding (BEZ rekompilacji). Z opcjonalnymi
             etykietami ground-truth generuje krzywą PR i ROC.

  sweep      Przemiatanie parametru wymagającego REKOMPILACJI (np. MIN_NEIGHBORS,
             TEMPORAL_WINDOW_US): dla każdej wartości buduje target `benchmark`
             z -D<MAKRO>=<val>, uruchamia headless na pliku .raw, zbiera metryki
             z summary JSON i rysuje metrykę vs wartość parametru.

Przykłady:
  python tools/param_sweep.py analyze output/metrics_benchmark.csv
  python tools/param_sweep.py ttc-sweep output/metrics_benchmark.csv \
         --labels tools/labels_example.csv
  python tools/param_sweep.py sweep --param OA_FILTER_MIN_NEIGHBORS \
         --values 1,2,3 --raw /home/sterydy/output/wykryty_ruch.raw
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ── tryb: analyze ────────────────────────────────────────────────────────────
def cmd_analyze(args):
    df = pd.read_csv(args.csv)
    outdir = args.outdir or os.path.join(PROJECT_ROOT, "output", "plots")
    os.makedirs(outdir, exist_ok=True)

    # 1) Latencja etapów vs liczba zdarzeń w slice (skalowanie O(N)).
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(df["slice_events"], df["tracker_us"], s=6, alpha=0.4, label="tracker")
    ax.scatter(df["slice_events"], df["ttc_us"], s=6, alpha=0.4, label="ttc")
    # liniowy fit dla trackera
    if len(df) > 2:
        k = np.polyfit(df["slice_events"], df["tracker_us"], 1)
        xs = np.linspace(df["slice_events"].min(), df["slice_events"].max(), 50)
        ax.plot(xs, np.polyval(k, xs), "r--",
                label=f"fit tracker: {k[0]*1000:.3f} ns/ev + {k[1]:.1f} µs")
    ax.set_xlabel("zdarzenia w slice (N)")
    ax.set_ylabel("latencja [µs]")
    ax.set_title("Latencja etapu vs liczba zdarzeń")
    ax.legend()
    ax.grid(True, alpha=0.3)
    p1 = os.path.join(outdir, "latency_vs_events.png")
    fig.savefig(p1, dpi=110, bbox_inches="tight")
    plt.close(fig)

    # 2) Rozkład latencji tracker+ttc (histogram + budżet 10 ms).
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(df["slice_total_us"], bins=60, color="steelblue", alpha=0.8)
    ax.axvline(10000, color="red", linestyle="--", label="budżet 10 ms")
    ax.set_xlabel("latencja tracker+ttc [µs]")
    ax.set_ylabel("liczba slice'ów")
    ax.set_title("Rozkład latencji przetwarzania slice")
    ax.legend()
    ax.grid(True, alpha=0.3)
    p2 = os.path.join(outdir, "latency_hist.png")
    fig.savefig(p2, dpi=110, bbox_inches="tight")
    plt.close(fig)

    # 3) Detekcje/kolizje w czasie + rozmiary buforów.
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    t = df["end_ts_us"] * 1e-6
    a1.plot(t, df["valid"], label="valid", color="green")
    a1.plot(t, df["danger"], label="danger", color="red", alpha=0.7)
    a1.set_ylabel("flaga")
    a1.legend(loc="upper right")
    a1.grid(True, alpha=0.3)
    a2.plot(t, df["filter_buffer"], label="bufor filtra", color="purple")
    a2.set_xlabel("czas [s]")
    a2.set_ylabel("zdarzenia w buforze")
    a2.legend()
    a2.grid(True, alpha=0.3)
    p3 = os.path.join(outdir, "detections_timeline.png")
    fig.savefig(p3, dpi=110, bbox_inches="tight")
    plt.close(fig)

    print(f"[OK] Zapisano wykresy:\n  {p1}\n  {p2}\n  {p3}")


# ── tryb: ttc-sweep (offline) ────────────────────────────────────────────────
def load_labels(path):
    """Zakresy czasowe [start_us, end_us] uznawane za realną kolizję."""
    ranges = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            a, b = line.replace(";", ",").split(",")[:2]
            ranges.append((float(a), float(b)))
    return ranges


def in_ranges(ts, ranges):
    return any(a <= ts <= b for a, b in ranges)


def cmd_ttc_sweep(args):
    df = pd.read_csv(args.csv)
    outdir = args.outdir or os.path.join(PROJECT_ROOT, "output", "plots")
    os.makedirs(outdir, exist_ok=True)

    thresholds = np.linspace(0.05, 1.0, 40)
    # Re-thresholding offline: danger = expanding & 0<ttc<T
    valid_ttc = (df["expanding"] == 1) & (df["ttc_value"] > 0)
    danger_counts = [int(((df["ttc_value"] < T) & valid_ttc).sum())
                     for T in thresholds]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(thresholds, danger_counts, "o-", color="darkorange")
    ax.axvline(0.45, color="gray", linestyle="--", label="domyślny próg 0.45 s")
    ax.set_xlabel("TTC_DANGER_THRESHOLD_S [s]")
    ax.set_ylabel("liczba slice'ów danger")
    ax.set_title("Liczba kolizji vs próg TTC (offline)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    p1 = os.path.join(outdir, "ttc_threshold_sweep.png")
    fig.savefig(p1, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Zapisano: {p1}")

    if not args.labels:
        print("[INFO] Brak --labels — pominięto krzywe PR/ROC.")
        return

    ranges = load_labels(args.labels)
    gt = df["end_ts_us"].apply(lambda ts: in_ranges(ts, ranges)).to_numpy()
    P = int(gt.sum())
    Nneg = int((~gt).sum())
    if P == 0 or Nneg == 0:
        print("[OSTRZEŻENIE] Etykiety nie dzielą danych na obie klasy — PR/ROC pominięte.")
        return

    prec, rec, tpr, fpr = [], [], [], []
    for T in thresholds:
        pred = ((df["ttc_value"] < T) & valid_ttc).to_numpy()
        tp = int((pred & gt).sum())
        fp = int((pred & ~gt).sum())
        fn = int((~pred & gt).sum())
        prec.append(tp / (tp + fp) if (tp + fp) else 1.0)
        rec.append(tp / (tp + fn) if (tp + fn) else 0.0)
        tpr.append(tp / P)
        fpr.append(fp / Nneg)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
    a1.plot(rec, prec, "o-", color="teal")
    a1.set_xlabel("recall")
    a1.set_ylabel("precision")
    a1.set_title("Krzywa PR (próg TTC)")
    a1.grid(True, alpha=0.3)
    a2.plot(fpr, tpr, "o-", color="crimson")
    a2.plot([0, 1], [0, 1], "k--", alpha=0.4)
    a2.set_xlabel("FPR")
    a2.set_ylabel("TPR")
    a2.set_title("Krzywa ROC (próg TTC)")
    a2.grid(True, alpha=0.3)
    p2 = os.path.join(outdir, "ttc_pr_roc.png")
    fig.savefig(p2, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Zapisano: {p2}")


# ── tryb: sweep (rekompilacja) ───────────────────────────────────────────────
def build_and_run(param, value, raw, base_build, source):
    build_dir = os.path.join(base_build, f"sweep_{param}_{value}")
    os.makedirs(build_dir, exist_ok=True)
    define = f"-D{param}={value}"
    cfg = subprocess.run(
        ["cmake", "-S", source, "-B", build_dir,
         f"-DCMAKE_CXX_FLAGS={define}"],
        capture_output=True, text=True)
    if cfg.returncode != 0:
        print(cfg.stdout, cfg.stderr)
        raise RuntimeError(f"cmake configure nie powiodło się dla {param}={value}")
    bld = subprocess.run(
        ["cmake", "--build", build_dir, "--target", "benchmark", "-j"],
        capture_output=True, text=True)
    if bld.returncode != 0:
        print(bld.stdout, bld.stderr)
        raise RuntimeError(f"build nie powiódł się dla {param}={value}")

    json_path = os.path.join(build_dir, "summary.json")
    csv_path = os.path.join(build_dir, "slices.csv")
    run = subprocess.run(
        [os.path.join(build_dir, "benchmark"), raw,
         "--metrics", csv_path, "--metrics-json", json_path,
         "--output-dir", build_dir],
        capture_output=True, text=True)
    if run.returncode != 0:
        print(run.stdout, run.stderr)
        raise RuntimeError(f"benchmark nie powiódł się dla {param}={value}")
    with open(json_path) as f:
        return json.load(f)


def cmd_sweep(args):
    values = [v.strip() for v in args.values.split(",") if v.strip()]
    base_build = args.build_dir or os.path.join(PROJECT_ROOT, "build_sweeps")
    outdir = args.outdir or os.path.join(PROJECT_ROOT, "output", "plots")
    os.makedirs(outdir, exist_ok=True)

    rows = []
    for v in values:
        print(f"[SWEEP] {args.param}={v} — buduję i uruchamiam...")
        s = build_and_run(args.param, v, args.raw, base_build, PROJECT_ROOT)
        rows.append({
            "value": float(v),
            "throughput_mev_s": s["throughput_mev_s"],
            "tracker_mean_us": s["tracker_us"]["mean"],
            "filter_mean_us": s["filter_us"]["mean"],
            "slice_p99_us": s["slice_total_us"]["p99"],
            "filter_rejection_pct": s["filter_rejection_pct"],
            "valid_slices": s["valid_slices"],
            "danger_slices": s["danger_slices"],
            "danger_episodes": s["danger_episodes"],
        })

    res = pd.DataFrame(rows).sort_values("value")
    csv_out = os.path.join(outdir, f"sweep_{args.param}.csv")
    res.to_csv(csv_out, index=False)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
    a1.plot(res["value"], res["throughput_mev_s"], "o-", label="throughput [Mev/s]")
    a1.plot(res["value"], res["tracker_mean_us"], "s-", label="tracker mean [µs]")
    a1.set_xlabel(args.param)
    a1.set_title("Wydajność vs parametr")
    a1.legend()
    a1.grid(True, alpha=0.3)
    a2.plot(res["value"], res["valid_slices"], "o-", color="green", label="detekcje")
    a2.plot(res["value"], res["danger_slices"], "s-", color="red", label="kolizje")
    a2.plot(res["value"], res["filter_rejection_pct"], "^-", color="purple",
            label="odrzut filtra [%]")
    a2.set_xlabel(args.param)
    a2.set_title("Jakość detekcji vs parametr")
    a2.legend()
    a2.grid(True, alpha=0.3)
    p = os.path.join(outdir, f"sweep_{args.param}.png")
    fig.savefig(p, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Zapisano: {csv_out}\n             {p}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("analyze", help="wykresy z pojedynczego CSV")
    a.add_argument("csv")
    a.add_argument("--outdir")
    a.set_defaults(func=cmd_analyze)

    t = sub.add_parser("ttc-sweep", help="offline'owy sweep progu TTC + PR/ROC")
    t.add_argument("csv")
    t.add_argument("--labels", help="plik z zakresami ground-truth (start_us,end_us)")
    t.add_argument("--outdir")
    t.set_defaults(func=cmd_ttc_sweep)

    s = sub.add_parser("sweep", help="sweep parametru z rekompilacją")
    s.add_argument("--param", required=True,
                   help="np. OA_FILTER_MIN_NEIGHBORS, OA_TEMPORAL_WINDOW_US, "
                        "OA_TTC_DANGER_THRESHOLD_S, OA_SLICE_DURATION_US")
    s.add_argument("--values", required=True, help="lista po przecinku, np. 1,2,3")
    s.add_argument("--raw", required=True, help="plik .raw do benchmarku")
    s.add_argument("--build-dir")
    s.add_argument("--outdir")
    s.set_defaults(func=cmd_sweep)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
