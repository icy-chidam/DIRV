"""
=============================================================
  IWR6843ISK — Run inference on a new CSV file
=============================================================
Usage:
    python run_inference.py your_new_data.csv

Output:
    • Prints predictions per frame to the terminal
    • Saves results to  results_<your_file>.csv
=============================================================
"""

import sys, os, pickle
import numpy as np
import pandas as pd

# ── Path to your trained model ────────────────────────────
MODEL_PATH = "radar_proximity_model.pkl"

# ── Cleaning thresholds (must match training) ─────────────
SNR_MIN   = 5.0
SNR_MAX   = 200.0
RANGE_MIN = 0.3
RANGE_MAX = 20.0
AZ_MAX    = 1.1


# ─────────────────────────────────────────────────────────
# STEP 1 — Load model
# ─────────────────────────────────────────────────────────
def load_model(model_path):
    if not os.path.exists(model_path):
        print(f"\n❌  Model not found: {model_path}")
        print("    Make sure 'radar_proximity_model.pkl' is in the same folder.")
        sys.exit(1)
    with open(model_path, "rb") as f:
        bundle = pickle.load(f)
    return bundle["pipeline"], bundle["label_encoder"], bundle["feature_cols"]


# ─────────────────────────────────────────────────────────
# STEP 2 — Clean the new CSV  (same steps as training)
# ─────────────────────────────────────────────────────────
def clean(df):
    df = df.drop_duplicates()
    df = df.dropna(subset=["x_m", "y_m", "z_m", "snr", "azimuth_rad"])

    df["range_m"] = np.sqrt(df["x_m"]**2 + df["y_m"]**2 + df["z_m"]**2).round(6)

    df = df[(df["snr"]     >= SNR_MIN)   & (df["snr"]     <= SNR_MAX)]
    df = df[(df["range_m"] >= RANGE_MIN) & (df["range_m"] <= RANGE_MAX)]
    df = df[df["azimuth_rad"].abs() <= AZ_MAX]

    # Remove frames with only 1 point
    frame_sizes = df.groupby("frame")["point_id"].transform("count")
    df = df[frame_sizes >= 2]

    return df.reset_index(drop=True)


# ─────────────────────────────────────────────────────────
# STEP 3 — Build per-frame features  (same as training)
# ─────────────────────────────────────────────────────────
def build_features(df):
    rows = []
    for frame_id, grp in df.groupby("frame"):
        if len(grp) < 2:
            continue

        x   = grp["x_m"].values
        y   = grp["y_m"].values
        z   = grp["z_m"].values
        snr = grp["snr"].values
        rng = grp["range_m"].values
        az  = grp["azimuth_rad"].values

        x_range = x.max() - x.min()
        y_range = y.max() - y.min()
        z_range = z.max() - z.min()
        bbox_vol  = (x_range+1e-6) * (y_range+1e-6) * (z_range+1e-6)
        bbox_area = (x_range+1e-6) * (y_range+1e-6)
        n = len(grp)
        cx, cy, cz = x.mean(), y.mean(), z.mean()

        rows.append({
            "frame":          frame_id,
            "x_mean":         cx,          "y_mean":       cy,
            "z_mean":         cz,          "x_std":        x.std(),
            "y_std":          y.std(),     "z_std":        z.std(),
            "x_range":        x_range,     "y_range":      y_range,
            "z_range":        z_range,     "bbox_volume":  bbox_vol,
            "bbox_area_xy":   bbox_area,
            "aspect_xy":      x_range / (y_range + 1e-6),
            "aspect_xz":      x_range / (z_range + 1e-6),
            "range_mean":     rng.mean(),  "range_std":    rng.std(),
            "range_min":      rng.min(),   "range_max":    rng.max(),
            "snr_mean":       snr.mean(),  "snr_std":      snr.std(),
            "snr_min":        snr.min(),   "snr_max":      snr.max(),
            "snr_range":      snr.max() - snr.min(),
            "az_std":         az.std(),    "az_range":     az.max() - az.min(),
            "z_spread":       z_range,
            "z_frac_above":   (z > 0.1).mean(),
            "n_points":       n,
            "centroid_dist":  np.sqrt(cx**2 + cy**2 + cz**2),
            "point_density":  n / (bbox_vol + 1e-6),
        })

    return pd.DataFrame(rows)


# ─────────────────────────────────────────────────────────
# STEP 4 — Predict + print results
# ─────────────────────────────────────────────────────────
def predict(feat_df, pipeline, le, feature_cols):
    X = feat_df[feature_cols]

    preds_enc   = pipeline.predict(X)
    preds_proba = pipeline.predict_proba(X)
    labels      = le.inverse_transform(preds_enc)

    result = feat_df[["frame"]].copy()
    result["predicted_class"] = labels
    result["confidence_%"]    = (preds_proba.max(axis=1) * 100).round(1)

    for i, cls in enumerate(le.classes_):
        result[f"prob_{cls}"] = (preds_proba[:, i] * 100).round(1)

    return result


# ─────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────
def main():
    # ── Get CSV path ──────────────────────────────────────
    if len(sys.argv) < 2:
        print("\nUsage:  python run_inference.py  your_file.csv\n")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"\n❌  File not found: {csv_path}\n")
        sys.exit(1)

    print("\n" + "=" * 55)
    print("  IWR6843ISK — Radar Inference")
    print("=" * 55)
    print(f"  Input  : {csv_path}")
    print(f"  Model  : {MODEL_PATH}")

    # ── Load ──────────────────────────────────────────────
    pipeline, le, feature_cols = load_model(MODEL_PATH)

    df_raw = pd.read_csv(csv_path)
    print(f"\n  Raw rows     : {len(df_raw):,}")
    print(f"  Raw frames   : {df_raw['frame'].nunique():,}")

    # ── Clean ─────────────────────────────────────────────
    df_clean = clean(df_raw)
    print(f"  Clean rows   : {len(df_clean):,}")
    print(f"  Clean frames : {df_clean['frame'].nunique():,}")

    if df_clean.empty:
        print("\n⚠️  No valid data after cleaning. Check your CSV format.")
        sys.exit(1)

    # ── Features ──────────────────────────────────────────
    feat_df = build_features(df_clean)
    print(f"  Feature rows : {len(feat_df):,}\n")

    # ── Predict ───────────────────────────────────────────
    results = predict(feat_df, pipeline, le, feature_cols)

    # ── Print summary table ───────────────────────────────
    ICONS = {"pedestrian": "🚶", "vehicle": "🚗", "static": "🪨"}
    print(f"  {'Frame':>6}  {'Prediction':12}  {'Confidence':>10}  {'Prob Ped%':>9}  {'Prob Veh%':>9}  {'Prob Sta%':>9}")
    print("  " + "-" * 65)

    for _, row in results.iterrows():
        cls  = row["predicted_class"]
        icon = ICONS.get(cls, "?")
        pp   = row.get("prob_pedestrian", 0)
        pv   = row.get("prob_vehicle", 0)
        ps   = row.get("prob_static", 0)
        print(f"  {int(row['frame']):>6}  {icon} {cls:11}  {row['confidence_%']:>9.1f}%  "
              f"{pp:>8.1f}%  {pv:>8.1f}%  {ps:>8.1f}%")

    # ── Summary ───────────────────────────────────────────
    print("\n" + "=" * 55)
    print("  SUMMARY")
    print("=" * 55)
    counts = results["predicted_class"].value_counts()
    total  = len(results)
    for cls, cnt in counts.items():
        icon = ICONS.get(cls, "?")
        bar  = "█" * int((cnt / total) * 30)
        print(f"  {icon} {cls:12}: {cnt:4d} frames  ({cnt/total*100:.1f}%)  {bar}")
    print(f"\n  Avg confidence : {results['confidence_%'].mean():.1f}%")

    # ── Save CSV ──────────────────────────────────────────
    out_name   = "results_" + os.path.basename(csv_path)
    out_path   = os.path.join(os.getcwd(), out_name)
    results.to_csv(out_path, index=False)
    print(f"\n  💾 Results saved → {out_path}")
    print("=" * 55 + "\n")


if __name__ == "__main__":
    main()
