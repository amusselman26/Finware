import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# =========================
# User settings
# =========================
CSV_PATH = "Pasted_text.csv"   # change if needed
SAVE_PLOTS = False             # True to save pngs
PLOT_DIR = "eskf_analysis"

# Optional tuning targets for a stationary test
TARGETS = {
    "pos_std_m_good": 0.5,
    "pos_std_m_ok": 1.5,
    "vel_abs_mean_good": 0.1,
    "vel_abs_mean_ok": 0.3,
    "att_std_deg_good": 0.2,
    "att_std_deg_ok": 1.0,
    "r_baro_std_good": 0.3,
    "r_baro_std_ok": 1.0,
    "r_gps_mean_good": 2.0,
    "r_gps_mean_ok": 5.0,
    "r_gps_vel_mean_good": 0.5,
    "r_gps_vel_mean_ok": 1.5,
}

# =========================
# Helpers
# =========================
def safe_print_stat(name, value, unit=""):
    if pd.isna(value):
        print(f"{name}: nan")
    else:
        print(f"{name}: {value:.4f} {unit}".strip())

def rolling_slope(t, y, window=40):
    """
    Approx local slope dy/dt using linear fit in a rolling window.
    Returns array same length as y, padded with NaN at edges.
    """
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    out = np.full_like(y, np.nan)

    half = window // 2
    for i in range(half, len(y) - half):
        ys = y[i-half:i+half]
        ts = t[i-half:i+half]
        if np.any(np.isnan(ys)) or np.any(np.isnan(ts)):
            continue
        p = np.polyfit(ts, ys, 1)
        out[i] = p[0]
    return out

def summarize_series(name, s):
    return {
        "name": name,
        "mean": np.nanmean(s),
        "std": np.nanstd(s),
        "min": np.nanmin(s),
        "max": np.nanmax(s),
        "range": np.nanmax(s) - np.nanmin(s),
        "abs_mean": np.nanmean(np.abs(s)),
        "rms": np.sqrt(np.nanmean(np.asarray(s)**2)),
    }

def recommend_tuning(stats):
    """
    Very simple rule-based recommendations for stationary tests.
    """
    recs = []

    # Velocity should be near zero when stationary
    if stats["v_z"]["abs_mean"] > TARGETS["vel_abs_mean_ok"]:
        recs.append(
            "- v_z is much too large for a stationary test. "
            "Increase trust in baro/GPS vertical correction or decrease vertical process trust. "
            "Likely actions: increase sigma_acc, reduce sigma_ba_rw, add/strengthen ZUPT when stationary, "
            "and verify gravity handling/sign conventions."
        )

    if stats["v_x"]["abs_mean"] > TARGETS["vel_abs_mean_ok"] or stats["v_y"]["abs_mean"] > TARGETS["vel_abs_mean_ok"]:
        recs.append(
            "- v_x / v_y are too large for stationary. "
            "Reduce horizontal drift by increasing sigma_acc, reducing sigma_ba_rw, "
            "adding a stationary zero-velocity update, and checking that GPS is not being over-applied."
        )

    # Position
    if stats["p_z"]["std"] > TARGETS["pos_std_m_ok"]:
        recs.append(
            "- p_z variability is high. "
            "If r_baro is noisy, increase baro measurement sigma. "
            "If p_z ramps while r_baro stays biased, decrease baro sigma slightly or reduce vertical process drift."
        )

    if stats["p_x"]["std"] > TARGETS["pos_std_m_ok"] or stats["p_y"]["std"] > TARGETS["pos_std_m_ok"]:
        recs.append(
            "- Horizontal position drift is high for stationary. "
            "Use realistic GPS noise, gate GPS outliers, and avoid letting GPS updates change attitude/bias states."
        )

    # Attitude
    if stats["roll_deg"]["std"] > TARGETS["att_std_deg_ok"] or stats["pitch_deg"]["std"] > TARGETS["att_std_deg_ok"]:
        recs.append(
            "- Roll/pitch noise is larger than expected at rest. "
            "Check IMU vibration/noise and consider increasing gyro measurement/process noise slightly."
        )

    if stats["yaw_deg"]["std"] > TARGETS["att_std_deg_ok"]:
        recs.append(
            "- Yaw is varying noticeably at rest. "
            "Without a heading reference, freeze yaw correction and z-gyro-bias correction while stationary."
        )

    # Residuals
    if "r_baro" in stats and stats["r_baro"]["std"] > TARGETS["r_baro_std_ok"]:
        recs.append(
            "- Baro residual is noisy. Increase baro measurement sigma or low-pass/filter baro before fusion."
        )

    if "r_gps_pos_norm" in stats and stats["r_gps_pos_norm"]["mean"] > TARGETS["r_gps_mean_ok"]:
        recs.append(
            "- GPS position residual norm is large on average. Increase GPS position measurement sigma, gate outliers, "
            "and verify your local NED conversion/reference."
        )

    if "r_gps_vel_norm" in stats and stats["r_gps_vel_norm"]["mean"] > TARGETS["r_gps_vel_mean_ok"]:
        recs.append(
            "- GPS velocity residual norm is large on average. Increase GPS velocity measurement sigma and "
            "check frame/sign consistency in velocity updates."
        )

    if "r_gps" in stats and stats["r_gps"]["mean"] > TARGETS["r_gps_mean_ok"]:
        recs.append(
            "- GPS residual is large on average. Increase GPS measurement sigma, gate outliers, "
            "and verify your local NED conversion/reference."
        )

    if not recs:
        recs.append("- No major red flags from the basic stationary metrics.")

    return recs

# =========================
# Load data
# =========================
path = Path(CSV_PATH)
if not path.exists():
    raise FileNotFoundError(f"Could not find file: {CSV_PATH}")

df = pd.read_csv(path)

required_cols = [
    "t_us", "p_x", "p_y", "p_z", "v_x", "v_y", "v_z",
    "roll_deg", "pitch_deg", "yaw_deg", "r_baro",
    "r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z",
    "r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z",
]
missing_cols = [c for c in required_cols if c not in df.columns]
if missing_cols:
    raise ValueError(
        f"Missing required columns for this tuner format: {missing_cols}"
    )

# Time in seconds from first sample
df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) * 1e-6

# Clean up NaNs
for col in df.columns:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Derived quantities
df["speed_norm"] = np.sqrt(df["v_x"]**2 + df["v_y"]**2 + df["v_z"]**2)
df["pos_norm"] = np.sqrt(df["p_x"]**2 + df["p_y"]**2 + df["p_z"]**2)

gps_pos_res_cols = ["r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z"]
if all(c in df.columns for c in gps_pos_res_cols):
    df["r_gps_pos_norm"] = np.sqrt(df["r_gps_pos_x"]**2 + df["r_gps_pos_y"]**2 + df["r_gps_pos_z"]**2)

gps_vel_res_cols = ["r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z"]
if all(c in df.columns for c in gps_vel_res_cols):
    df["r_gps_vel_norm"] = np.sqrt(df["r_gps_vel_x"]**2 + df["r_gps_vel_y"]**2 + df["r_gps_vel_z"]**2)

# Rolling drift rates
for col in ["p_x", "p_y", "p_z", "v_x", "v_y", "v_z"]:
    df[f"{col}_slope"] = rolling_slope(df["t_s"], df[col], window=40)

# =========================
# Stats
# =========================
state_cols = [
    "p_x", "p_y", "p_z",
    "v_x", "v_y", "v_z",
    "roll_deg", "pitch_deg", "yaw_deg",
    "r_baro",
    "r_gps",
    "r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z",
    "r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z",
    "r_gps_pos_norm", "r_gps_vel_norm",
]

stats = {}
for c in state_cols:
    if c in df.columns:
        stats[c] = summarize_series(c, df[c])

print("\n===== BASIC SUMMARY =====")
print(f"Samples: {len(df)}")
safe_print_stat("Duration", df["t_s"].iloc[-1], "s")
safe_print_stat("Mean dt", df["t_s"].diff().mean(), "s")
safe_print_stat("Mean loop rate", 1.0 / df["t_s"].diff().mean(), "Hz")

print("\n===== STATE STATS =====")
for c in state_cols:
    if c in stats:
        print(f"\n{c}")
        safe_print_stat("  mean", stats[c]["mean"])
        safe_print_stat("  std", stats[c]["std"])
        safe_print_stat("  min", stats[c]["min"])
        safe_print_stat("  max", stats[c]["max"])
        safe_print_stat("  range", stats[c]["range"])
        safe_print_stat("  abs_mean", stats[c]["abs_mean"])
        safe_print_stat("  rms", stats[c]["rms"])

print("\n===== DRIFT RATES (median rolling slope) =====")
for c in ["p_x", "p_y", "p_z", "v_x", "v_y", "v_z"]:
    if f"{c}_slope" in df.columns:
        med_slope = np.nanmedian(df[f"{c}_slope"])
        safe_print_stat(f"{c} drift rate", med_slope, f"{c.split('_')[0]}/s")

# =========================
# Stationary health checks
# =========================
print("\n===== STATIONARY HEALTH CHECKS =====")
print("Expected for stationary test:")
print("- position roughly bounded")
print("- velocity near zero")
print("- roll/pitch/yaw bounded")
print("- residuals approximately zero-mean")

speed_mean = df["speed_norm"].mean()
speed_std = df["speed_norm"].std()
safe_print_stat("Mean speed norm", speed_mean, "m/s")
safe_print_stat("Std speed norm", speed_std, "m/s")

# =========================
# Tuning recommendations
# =========================
print("\n===== TUNING RECOMMENDATIONS =====")
for rec in recommend_tuning(stats):
    print(rec)

# More explicit parameter guidance
print("\n===== PARAMETER DIRECTION GUIDE =====")
print("Use these as directional rules:")
print("- If residuals are noisy but estimate chases them -> increase measurement sigma R.")
print("- If estimate drifts too much between updates -> increase process noise on that propagation path OR add stationary constraints.")
print("- If bias estimates wander while stationary -> reduce sigma_bg_rw / sigma_ba_rw.")
print("- If GPS causes jumps -> increase GPS sigma and add innovation gating.")
print("- If vertical velocity is nonzero while stationary -> add a zero-velocity update (ZUPT) when stationary.")

# =========================
# Plotting
# =========================
if SAVE_PLOTS:
    Path(PLOT_DIR).mkdir(parents=True, exist_ok=True)

# 1) Position
plt.figure(figsize=(10, 6))
plt.plot(df["t_s"], df["p_x"], label="p_x")
plt.plot(df["t_s"], df["p_y"], label="p_y")
plt.plot(df["t_s"], df["p_z"], label="p_z")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("Position vs Time")
plt.grid(True)
plt.legend()
if SAVE_PLOTS:
    plt.savefig(f"{PLOT_DIR}/position.png", dpi=150)

# 2) Velocity
plt.figure(figsize=(10, 6))
plt.plot(df["t_s"], df["v_x"], label="v_x")
plt.plot(df["t_s"], df["v_y"], label="v_y")
plt.plot(df["t_s"], df["v_z"], label="v_z")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Velocity vs Time")
plt.grid(True)
plt.legend()
if SAVE_PLOTS:
    plt.savefig(f"{PLOT_DIR}/velocity.png", dpi=150)

# 3) Attitude
plt.figure(figsize=(10, 6))
plt.plot(df["t_s"], df["roll_deg"], label="roll")
plt.plot(df["t_s"], df["pitch_deg"], label="pitch")
plt.plot(df["t_s"], df["yaw_deg"], label="yaw")
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.title("Attitude vs Time")
plt.grid(True)
plt.legend()
if SAVE_PLOTS:
    plt.savefig(f"{PLOT_DIR}/attitude.png", dpi=150)

# 4) Residuals
plt.figure(figsize=(10, 6))
if "r_baro" in df.columns:
    plt.plot(df["t_s"], df["r_baro"], label="r_baro")
if "r_gps" in df.columns:
    plt.plot(df["t_s"], df["r_gps"], label="r_gps")
for col in ["r_gps_pos_x", "r_gps_pos_y", "r_gps_pos_z", "r_gps_vel_x", "r_gps_vel_y", "r_gps_vel_z"]:
    if col in df.columns:
        plt.plot(df["t_s"], df[col], label=col)
if "r_gps_pos_norm" in df.columns:
    plt.plot(df["t_s"], df["r_gps_pos_norm"], label="r_gps_pos_norm")
if "r_gps_vel_norm" in df.columns:
    plt.plot(df["t_s"], df["r_gps_vel_norm"], label="r_gps_vel_norm")
plt.xlabel("Time [s]")
plt.ylabel("Residual")
plt.title("Measurement Residuals vs Time")
plt.grid(True)
plt.legend()
if SAVE_PLOTS:
    plt.savefig(f"{PLOT_DIR}/residuals.png", dpi=150)

# 5) Histograms
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.flatten()

for ax, col in zip(axes, ["p_z", "v_z", "r_baro", "r_gps_pos_norm"]):
    if col in df.columns:
        ax.hist(df[col].dropna(), bins=40)
        ax.set_title(f"{col} histogram")
        ax.grid(True)

if "r_gps_pos_norm" not in df.columns and "r_gps" in df.columns:
    axes[3].hist(df["r_gps"].dropna(), bins=40)
    axes[3].set_title("r_gps histogram")
    axes[3].grid(True)

if "r_gps_vel_norm" in df.columns:
    fig2, ax2 = plt.subplots(1, 1, figsize=(6, 4))
    ax2.hist(df["r_gps_vel_norm"].dropna(), bins=40)
    ax2.set_title("r_gps_vel_norm histogram")
    ax2.grid(True)

plt.tight_layout()
if SAVE_PLOTS:
    plt.savefig(f"{PLOT_DIR}/histograms.png", dpi=150)

plt.show()