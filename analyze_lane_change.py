"""Shared lane-change plots; analysis requires no PyChrono installation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from analyze_cornering import (
    AXIS,
    CRITICAL,
    ESTIMATE_COLOR,
    MODEL_COLOR,
    ROOT,
    SIM_COLOR,
    WHEEL_COLORS,
    WHEEL_LABELS,
    _apply_style,
    _conditions,
    _figure_prefix,
    _summary_path,
    _title,
    linear_model_response,
    plot_lateral_response,
    plot_tires,
    plt,
)


def _mark_phases(ax, summary: dict[str, Any] | None, end_time_s: float):
    profile = (summary or {}).get("steering_input", {})
    times = {profile.get(key) for key in ("start_time_s", "return_start_time_s", "end_time_s")}
    if "start_time_s" in profile and "duration_per_lane_change_s" in profile:
        times.add(profile["start_time_s"] + profile["duration_per_lane_change_s"])
    for time in sorted(time for time in times if time is not None):
        if time <= end_time_s:
            ax.axvline(time, color=AXIS, lw=0.8, linestyle="--", zorder=1)


def plot_inputs(df: pd.DataFrame, summary: dict[str, Any] | None, out: Path):
    fig, axes = plt.subplots(3, 1, figsize=(8.6, 7.6), sharex=True)
    fig.subplots_adjust(left=0.11, right=0.97, top=0.86, bottom=0.08, hspace=0.43)
    _title(fig, "Lane-change steering input and speed hold", _conditions(summary, df))
    time = df["time"]
    for ax in axes:
        _mark_phases(ax, summary, float(time.iloc[-1]))

    ax = axes[0]
    ax.plot(time, df["steer_cmd_deg"], color=ESTIMATE_COLOR, lw=2.1, label="Command")
    ax.plot(time, df["steer_angle_deg"], color=SIM_COLOR, lw=1.4, linestyle="--", label="Actual")
    ax.axhline(0.0, color=AXIS, lw=0.8)
    ax.set_title("Front-wheel angle (+ = left)")
    ax.set_ylabel("Steering (deg)")
    ax.legend(loc="upper right", ncols=2)

    ax = axes[1]
    ax.plot(time, df["speed_kph"], color=SIM_COLOR, label="Actual")
    if summary:
        ax.axhline(summary["target_speed_kph"], color=MODEL_COLOR, lw=1.2, linestyle="--", label="Target")
    ax.set_title("Vehicle speed")
    ax.set_ylabel("Speed (km/h)")
    ax.legend(loc="best", ncols=2)

    ax = axes[2]
    ax.plot(time, df["drive_torque_Nm"], color=SIM_COLOR)
    ax.set_title("Total drive torque (equal split between driven wheels)")
    ax.set_ylabel("Torque (N m)")
    ax.set_xlabel("Time (s)")
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_rollover(
    df: pd.DataFrame,
    linear: pd.DataFrame | None,
    summary: dict[str, Any] | None,
    out: Path,
):
    fig, axes = plt.subplots(3, 1, figsize=(8.6, 8.7), sharex=True)
    fig.subplots_adjust(left=0.11, right=0.97, top=0.86, bottom=0.08, hspace=0.65)
    _title(fig, "Lane-change rollover response", _conditions(summary, df))
    time = df["time"]
    for ax in axes:
        _mark_phases(ax, summary, float(time.iloc[-1]))
        if summary and summary.get("wheel_lift_time_s") is not None:
            ax.axvline(summary["wheel_lift_time_s"], color=CRITICAL, lw=1.0)

    ax = axes[0]
    ax.plot(time, df["RI"], color=SIM_COLOR, label="Rear-load RI")
    ax.plot(time, df["RI_model"], color=ESTIMATE_COLOR, label="Acceleration estimate (Eq. 24)")
    if linear is not None:
        ax.plot(time, linear["RI"], color=MODEL_COLOR, linestyle="--", label="Linear model (Eq. 27)")
    for limit in (-1.0, 1.0):
        ax.axhline(limit, color=CRITICAL, lw=0.9, linestyle=":")
    ax.set_title("Rollover index (RI = +1: rear left unloaded; -1: rear right unloaded)")
    ax.set_ylabel("RI (-)")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.12), ncols=3, fontsize=8)

    weight = None
    if summary:
        weight = summary["total_mass_kg"] * summary["vehicle_parameters"]["g_mps2"]
    ax = axes[1]
    for wheel in ("Wheel_RL", "Wheel_RR"):
        values = df[f"{wheel}_contact_Fz_N"]
        ax.plot(time, values / weight if weight else values, color=WHEEL_COLORS[wheel], label=WHEEL_LABELS[wheel])
    ax.axhline(0.0, color=CRITICAL, lw=0.9, linestyle=":")
    ax.set_title("Rear-wheel vertical loads")
    ax.set_ylabel("Fz / mg (-)" if weight else "Fz (N)")
    ax.legend(loc="best", ncols=2)

    ax = axes[2]
    ax.plot(time, df["roll_deg"], color=SIM_COLOR)
    ax.set_title("Chassis roll angle (+ = right lean)")
    ax.set_ylabel("Roll (deg)")
    ax.set_xlabel("Time (s)")
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_trajectory(df: pd.DataFrame, summary: dict[str, Any] | None, out: Path):
    fig, axes = plt.subplots(2, 1, figsize=(8.6, 7.2))
    fig.subplots_adjust(left=0.11, right=0.97, top=0.86, bottom=0.09, hspace=0.52)
    _title(fig, "Lane-change trajectory", _conditions(summary, df))
    x = df["CoG_x"] - df["CoG_x"].iloc[0]
    y = df["CoG_y"] - df["CoG_y"].iloc[0]
    ax = axes[0]
    ax.plot(x, y, color=SIM_COLOR, label="CoG path")
    ax.axhline(0.0, color=MODEL_COLOR, linestyle="--", lw=1.0, label="Initial lateral position")
    ax.plot(x.iloc[0], y.iloc[0], "o", color=SIM_COLOR, ms=5)
    ax.plot(x.iloc[-1], y.iloc[-1], "s", color=SIM_COLOR, ms=5)
    ax.set_title("Plan view (equal distance scale)")
    ax.set_xlabel("Longitudinal displacement (m)")
    ax.set_ylabel("Lateral displacement (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.0), ncols=2, fontsize=8)

    ax = axes[1]
    ax.plot(df["time"], y, color=SIM_COLOR)
    ax.axhline(0.0, color=MODEL_COLOR, linestyle="--", lw=1.0)
    _mark_phases(ax, summary, float(df["time"].iloc[-1]))
    ax.set_title(f"Lateral displacement (final {y.iloc[-1]:+.3f} m)")
    ax.set_ylabel("Lateral displacement (m)")
    ax.set_xlabel("Time (s)")
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _validate_telemetry(df: pd.DataFrame, csv_path: Path):
    required = {
        "time", "steer_cmd_deg", "steer_angle_deg", "speed_kph", "v_cog_mps",
        "drive_torque_Nm", "beta_deg", "yaw_rate_dps", "ay_g", "roll_deg",
        "RI", "RI_model", "CoG_x", "CoG_y",
    }
    for wheel in ("Wheel_F", "Wheel_RL", "Wheel_RR"):
        required.update(f"{wheel}_{suffix}" for suffix in ("contact_Fz_N", "Fy_wheel_N", "slip_angle_deg"))
    missing = sorted(required.difference(df.columns))
    if missing:
        raise ValueError(f"{csv_path.name} is missing telemetry columns: {', '.join(missing)}")
    if len(df) < 2:
        raise ValueError(f"{csv_path.name} needs at least two telemetry samples.")
    numeric = df[sorted(required)].apply(pd.to_numeric, errors="raise")
    # RI is undefined when both rear wheels lose contact; allow NaN only there.
    finite_columns = numeric.drop(columns="RI")
    if not np.isfinite(finite_columns.to_numpy()).all() or np.isinf(numeric["RI"]).any():
        raise ValueError(f"{csv_path.name} contains nonfinite telemetry values.")
    if not (np.diff(numeric["time"].to_numpy()) > 0.0).all():
        raise ValueError("Telemetry times must be strictly increasing.")


def analyze(csv_path: Path, maneuver: str) -> list[Path]:
    csv_path = Path(csv_path)
    if not csv_path.is_absolute():
        csv_path = csv_path.resolve() if csv_path.exists() else ROOT / csv_path
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing {csv_path}. Run {maneuver}.py first.")
    df = pd.read_csv(csv_path)
    _validate_telemetry(df, csv_path)

    summary_file = _summary_path(csv_path)
    summary = None
    if summary_file.exists():
        summary = json.loads(summary_file.read_text(encoding="utf-8"))
        if summary.get("test") != maneuver:
            raise ValueError(f"Expected {maneuver} telemetry, but summary identifies {summary.get('test')!r}.")
    else:
        print(f"Summary missing: {summary_file.name}; plotting CSV without linear-model comparison.")
    linear = linear_model_response(df, summary["vehicle_parameters"]) if summary else None

    _apply_style()
    prefix = _figure_prefix(csv_path)
    outputs = []
    plots = {
        "inputs": plot_inputs,
        "lateral_response": lambda d, s, o: plot_lateral_response(d, linear, s, o),
        "rollover": lambda d, s, o: plot_rollover(d, linear, s, o),
        "tires": plot_tires,
        "trajectory": plot_trajectory,
    }
    for name, plot in plots.items():
        output = prefix.with_name(f"{prefix.name}_{name}.png")
        plot(df, summary, output)
        outputs.append(output)
        print(f"Plot written: {output.resolve()}")

    print(f"\n{maneuver.replace('_', ' ').capitalize()}")
    print(f"  Final lateral displacement: {df['CoG_y'].iloc[-1] - df['CoG_y'].iloc[0]:+.3f} m")
    print(f"  Peak lateral acceleration: {df['ay_g'].abs().max():.3f} g")
    print(f"  Peak |RI|: {df['RI'].abs().max():.3f}")
    if summary:
        print(f"  Steering input completed: {summary['maneuver_completed']}")
        print(f"  Final heading change: {summary['final_heading_change_deg']:+.3f} deg")
        if summary.get("wheel_lift_time_s") is not None:
            print(f"  Rear wheel lift: {summary['lifted_wheel']} at {summary['wheel_lift_time_s']:.3f} s")
        if summary.get("rolled_over"):
            print(f"  ROLLED OVER at {summary['rollover_time_s']:.3f} s")
    return outputs


def main(maneuver: str):
    default_csv = ROOT / "results" / f"{maneuver}_telemetry.csv"
    parser = argparse.ArgumentParser(description=f"Plot {maneuver.replace('_', ' ')} telemetry and the paper's linear-model response.")
    parser.add_argument("csv", nargs="?", default=str(default_csv), help="Telemetry CSV path; the sibling summary JSON is optional.")
    args = parser.parse_args()
    analyze(Path(args.csv), maneuver)
