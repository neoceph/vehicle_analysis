from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import font_manager


ROOT = Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "results" / "cornering_constant_steer_telemetry.csv"

# Chart chrome and categorical slots 1-3 of the reference data-viz palette.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
WINDOW = "#f0efec"
CRITICAL = "#d03b3b"
DANGER_ALPHA = 0.14
SERIES = ("#2a78d6", "#eb6834", "#1baf7a")

SIM_COLOR = SERIES[0]
ESTIMATE_COLOR = SERIES[1]
MODEL_COLOR = MUTED

WHEELS = ("Wheel_F", "Wheel_RL", "Wheel_RR")
WHEEL_LABELS = {"Wheel_F": "Front", "Wheel_RL": "Rear left", "Wheel_RR": "Rear right"}
WHEEL_COLORS = dict(zip(WHEELS, SERIES))


def _summary_path(csv_path: Path) -> Path:
    """Sibling summary JSON of a telemetry CSV (…_telemetry.csv -> …_summary.json)."""
    stem = csv_path.stem
    if stem.endswith("_telemetry"):
        stem = stem[: -len("_telemetry")]
    return csv_path.with_name(f"{stem}_summary.json")


def _figure_prefix(csv_path: Path) -> Path:
    stem = csv_path.stem
    if stem.endswith("_telemetry"):
        stem = stem[: -len("_telemetry")]
    return csv_path.with_name(stem)


def _apply_style():
    available = {font.name for font in font_manager.fontManager.ttflist}
    family = next(
        (name for name in ("Segoe UI", "Helvetica Neue", "Arial") if name in available),
        "DejaVu Sans",
    )
    plt.rcParams.update(
        {
            "font.family": family,
            "font.size": 9.5,
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": AXIS,
            "axes.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "grid.linestyle": "-",
            "axes.axisbelow": True,
            "axes.labelcolor": INK_SECONDARY,
            "axes.titlecolor": INK,
            "axes.titlesize": 10.5,
            "axes.titleweight": "semibold",
            "axes.titlelocation": "left",
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "xtick.labelcolor": INK_SECONDARY,
            "ytick.labelcolor": INK_SECONDARY,
            "lines.linewidth": 1.7,
            "lines.solid_capstyle": "round",
            "lines.solid_joinstyle": "round",
        }
    )


# ----------------------------------------------------------------------
# Linear lateral model of Rodriguez Licea et al. (2018), Eqs. (14), (16),
# (26), (27), in matrix form (Eq. 30) with constant cornering stiffness.
# ----------------------------------------------------------------------
def _state_matrices(p: dict[str, float], speed: float):
    m, inertia = p["m_kg"], p["J_kg_m2"]
    l_f, l_r = p["l_f_m"], p["l_r_m"]
    c_f, c_r = p["c_f_N_per_rad"], p["c_r_N_per_rad"]
    a = np.array(
        [
            [-(c_f + c_r) / (m * speed), (c_r * l_r - c_f * l_f) / (m * speed**2) - 1.0],
            [(c_r * l_r - c_f * l_f) / inertia, -(c_r * l_r**2 + c_f * l_f**2) / (inertia * speed)],
        ]
    )
    b = np.array([c_f / (m * speed), l_f * c_f / inertia])
    return a, b


def _lateral_acceleration(p: dict[str, float], speed, beta, yaw_rate, steer):
    """Eq. (26): a_y = (F_yf + F_yr) / m with linear tire forces."""
    m, l_f, l_r = p["m_kg"], p["l_f_m"], p["l_r_m"]
    c_f, c_r = p["c_f_N_per_rad"], p["c_r_N_per_rad"]
    return (
        -(c_f + c_r) / m * beta
        + (c_r * l_r - c_f * l_f) / (m * speed) * yaw_rate
        + c_f / m * steer
    )


def _rollover_index(p: dict[str, float], ay):
    """Eq. (24)/(27) with RI = +1 when the left rear wheel is unloaded."""
    gain = 2.0 * p["h_m"] * p["l_m"] / (p["b_m"] * p["g_mps2"] * p["l_f_m"])
    offset = p["l_m"] * (p["b_r_m"] - p["b_l_m"]) / (p["b_m"] * p["l_f_m"])
    return gain * ay - offset


def linear_model_response(df: pd.DataFrame, p: dict[str, float]) -> pd.DataFrame:
    """Integrate the linear model (RK4) with the simulated steer angle and speed.

    The speed is a time-varying parameter, as in the paper's LPV treatment.
    """
    time = df["time"].to_numpy()
    steer = np.radians(df["steer_angle_deg"].to_numpy())
    speed = np.maximum(df["v_cog_mps"].to_numpy(), 0.5)

    def derivative(state, v, delta):
        a, b = _state_matrices(p, v)
        return a @ state + b * delta

    states = np.zeros((len(time), 2))
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        if dt <= 0.0:
            states[i] = states[i - 1]
            continue
        a, _ = _state_matrices(p, min(speed[i - 1], speed[i]))
        substeps = max(1, math.ceil(dt * (abs(a[0, 0]) + abs(a[1, 1])) / 0.5))
        h = dt / substeps
        state = states[i - 1]
        for k in range(substeps):
            f0, f1 = k / substeps, (k + 0.5) / substeps
            f2 = (k + 1) / substeps
            v0, v1, v2 = (speed[i - 1] + f * (speed[i] - speed[i - 1]) for f in (f0, f1, f2))
            d0, d1, d2 = (steer[i - 1] + f * (steer[i] - steer[i - 1]) for f in (f0, f1, f2))
            k1 = derivative(state, v0, d0)
            k2 = derivative(state + 0.5 * h * k1, v1, d1)
            k3 = derivative(state + 0.5 * h * k2, v1, d1)
            k4 = derivative(state + h * k3, v2, d2)
            state = state + h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        states[i] = state

    beta, yaw_rate = states[:, 0], states[:, 1]
    ay = _lateral_acceleration(p, speed, beta, yaw_rate, steer)
    return pd.DataFrame(
        {
            "time": time,
            "beta_deg": np.degrees(beta),
            "yaw_rate_dps": np.degrees(yaw_rate),
            "ay_mps2": ay,
            "ay_g": ay / p["g_mps2"],
            "RI": _rollover_index(p, ay),
        }
    )


def linear_steady_state(p: dict[str, float], speed: float, steer_rad: float) -> dict[str, float]:
    a, b = _state_matrices(p, speed)
    beta, yaw_rate = np.linalg.solve(a, -b * steer_rad)
    ay = _lateral_acceleration(p, speed, beta, yaw_rate, steer_rad)
    understeer = (p["m_kg"] / p["l_m"]) * (
        p["l_r_m"] / p["c_f_N_per_rad"] - p["l_f_m"] / p["c_r_N_per_rad"]
    )
    return {
        "yaw_rate_dps": math.degrees(yaw_rate),
        "lateral_acceleration_g": ay / p["g_mps2"],
        "sideslip_deg": math.degrees(beta),
        "turn_radius_m": abs(speed / yaw_rate) if abs(yaw_rate) > 1.0e-6 else None,
        "RI": float(_rollover_index(p, ay)),
        "understeer_gradient_deg_per_g": math.degrees(understeer) * p["g_mps2"],
    }


# ----------------------------------------------------------------------
# Figures
# ----------------------------------------------------------------------
def _legend(ax):
    legend = ax.legend(
        loc="best",
        ncols=1,
        fontsize=8.5,
        labelcolor=INK_SECONDARY,
        frameon=True,
        facecolor=SURFACE,
        edgecolor=GRID,
        framealpha=0.9,
        fancybox=False,
        borderaxespad=0.6,
        borderpad=0.5,
        labelspacing=0.4,
        handlelength=2.0,
        handletextpad=0.7,
    )
    legend.get_frame().set_linewidth(0.6)
    return legend


def _shade_window(ax, summary: dict[str, Any] | None):
    steady = (summary or {}).get("steady_state")
    if steady:
        start, end = steady["window_s"]
        ax.axvspan(start, end, color=WINDOW, zorder=0, lw=0)


def _shade_ri_danger_zones(ax):
    ymin, ymax = ax.get_ylim()
    if ymax > 1.0:
        ax.axhspan(1.0, ymax, color=CRITICAL, alpha=DANGER_ALPHA, zorder=0.1, lw=0)
    if ymin < -1.0:
        ax.axhspan(ymin, -1.0, color=CRITICAL, alpha=DANGER_ALPHA, zorder=0.1, lw=0)
    ax.set_ylim(ymin, ymax)


def _window_note(fig, summary: dict[str, Any] | None):
    steady = (summary or {}).get("steady_state")
    if steady:
        start, end = steady["window_s"]
        fig.text(
            0.99,
            0.005,
            f"Shaded: steady-state averaging window ({start:.1f}-{end:.1f} s)",
            ha="right",
            va="bottom",
            color=MUTED,
            fontsize=8,
        )


def _title(fig, title: str, subtitle: str):
    fig.suptitle(title, x=0.01, ha="left", fontsize=12.5, fontweight="semibold", color=INK)
    fig.text(0.01, 0.955, subtitle, ha="left", va="top", color=INK_SECONDARY, fontsize=9.5)


def _conditions(summary: dict[str, Any] | None, df: pd.DataFrame) -> str:
    if summary is None:
        return f"Run length {df['time'].iloc[-1]:.1f} s"
    if summary.get("test") in ("single_lane_change", "double_lane_change"):
        return (
            f"{summary['target_speed_kph']:.1f} km/h, prescribed front-wheel angle "
            f"{summary['steering_angle_deg']:+.1f} deg peak, road friction "
            f"{summary['road_friction']:.2f}, {summary['total_mass_kg']:.0f} kg"
        )
    side = "left" if summary["steering_angle_deg"] >= 0.0 else "right"
    return (
        f"{summary['target_speed_kph']:.0f} km/h, constant front-wheel steer "
        f"{abs(summary['steering_angle_deg']):.1f}° ({side} turn) from "
        f"t = {summary['steering_start_time_s']:.1f} s, road friction "
        f"{summary['road_friction']:.2f}, {summary['total_mass_kg']:.0f} kg"
    )


def _end_label(ax, x, y, text):
    ax.annotate(
        text,
        xy=(x, y),
        xytext=(6, 0),
        textcoords="offset points",
        va="center",
        ha="left",
        color=INK_SECONDARY,
        fontsize=8.5,
    )


def plot_inputs(df: pd.DataFrame, summary: dict[str, Any] | None, out: Path):
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.2), sharex=True)
    fig.subplots_adjust(left=0.1, right=0.84, top=0.88, bottom=0.08, hspace=0.38)
    _title(fig, "Steering input and speed hold", _conditions(summary, df))
    time = df["time"]

    ax = axes[0]
    _shade_window(ax, summary)
    ax.plot(time, df["steer_angle_deg"], color=SIM_COLOR)
    ax.set_title("Front-wheel steer angle")
    ax.set_ylabel("Steer angle (°)")
    _end_label(ax, time.iloc[-1], df["steer_angle_deg"].iloc[-1], f"{df['steer_angle_deg'].iloc[-1]:.2f}°")

    ax = axes[1]
    _shade_window(ax, summary)
    if summary is not None:
        target = summary["target_speed_kph"]
        ax.axhline(target, color=MUTED, lw=0.9, zorder=1)
        ax.annotate(
            f"target {target:.0f} km/h",
            xy=(time.iloc[0], target),
            xytext=(4, 5),
            textcoords="offset points",
            color=INK_SECONDARY,
            fontsize=8.5,
        )
    ax.plot(time, df["speed_kph"], color=SIM_COLOR)
    ax.set_title("Vehicle speed (PI speed hold)")
    ax.set_ylabel("Speed (km/h)")
    spread = max(float(df["speed_kph"].max() - df["speed_kph"].min()), 0.5)
    middle = float(df["speed_kph"].median())
    ax.set_ylim(middle - 1.5 * spread, middle + 1.5 * spread)

    ax = axes[2]
    _shade_window(ax, summary)
    ax.plot(time, df["drive_torque_Nm"], color=SIM_COLOR)
    drive_wheels = (
        " + ".join(WHEEL_LABELS.get(w, w).lower() for w in summary["drive_wheels"])
        if summary
        else "driven wheels"
    )
    ax.set_title(f"Total drive torque ({drive_wheels}, equal split)")
    ax.set_ylabel("Torque (N·m)")
    ax.set_xlabel("Time (s)")
    _end_label(ax, time.iloc[-1], df["drive_torque_Nm"].iloc[-1], f"{df['drive_torque_Nm'].iloc[-1]:.0f} N·m")

    _window_note(fig, summary)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_lateral_response(
    df: pd.DataFrame,
    linear: pd.DataFrame | None,
    summary: dict[str, Any] | None,
    out: Path,
):
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.6), sharex=True)
    fig.subplots_adjust(left=0.1, right=0.97, top=0.86, bottom=0.08, hspace=0.38)
    _title(
        fig,
        "Lateral response: multibody simulation vs linear model",
        "Linear model: Rodriguez Licea et al. (2018), Eqs. (14), (16), (26), "
        "driven by the simulated steer angle and speed",
    )
    time = df["time"]
    panels = (
        ("beta_deg", "Vehicle sideslip angle β at the CoG", "β (°)", 1.0),
        ("yaw_rate_dps", "Yaw rate", "Yaw rate (°/s)", 1.0),
        ("ay_g", "Lateral acceleration at the CoG", "a_y (g)", 1.0),
    )
    for ax, (column, title, label, _) in zip(axes, panels):
        _shade_window(ax, summary)
        ax.plot(time, df[column], color=SIM_COLOR, label="Multibody simulation (TMeasy tires)")
        if linear is not None:
            ax.plot(
                linear["time"],
                linear[column],
                color=MODEL_COLOR,
                lw=1.6,
                linestyle=(0, (5, 3)),
                label="Linear model (paper)",
            )
        ax.set_title(title)
        ax.set_ylabel(label)
    _legend(axes[0])
    axes[-1].set_xlabel("Time (s)")
    _window_note(fig, summary)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_rollover(
    df: pd.DataFrame,
    linear: pd.DataFrame | None,
    summary: dict[str, Any] | None,
    out: Path,
):
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(8.0, 6.6),
        sharex=True,
        gridspec_kw={"height_ratios": [1.7, 1.0]},
    )
    fig.subplots_adjust(left=0.1, right=0.97, top=0.86, bottom=0.09, hspace=0.32)
    _title(
        fig,
        "Rollover index",
        "RI = (Fz_RR − Fz_RL) / (Fz_RR + Fz_RL): +1 = left rear wheel unloaded "
        "(rollover to the right), −1 = right rear wheel unloaded",
    )
    time = df["time"]

    ax = axes[0]
    _shade_window(ax, summary)
    for limit, offset, text in ((1.0, 4, "RI = +1: left rear lifts"), (-1.0, -12, "RI = −1: right rear lifts")):
        ax.axhline(limit, color=CRITICAL, lw=0.9, zorder=1)
        ax.annotate(
            text,
            xy=(time.iloc[-1], limit),
            xytext=(0, offset),
            textcoords="offset points",
            ha="right",
            color=INK_SECONDARY,
            fontsize=8.5,
        )
    ax.axhline(0.0, color=AXIS, lw=0.8, zorder=1)
    ax.set_title("Rollover index: simulation vs the paper's estimators")
    ax.plot(time, df["RI"], color=SIM_COLOR, label="Simulation, from rear vertical loads (Eq. 17)")
    ax.plot(
        time,
        df["RI_model"],
        color=ESTIMATE_COLOR,
        lw=1.4,
        label="Estimate from simulated a_y (Eq. 24)",
    )
    if linear is not None:
        ax.plot(
            linear["time"],
            linear["RI"],
            color=MODEL_COLOR,
            lw=1.6,
            linestyle=(0, (5, 3)),
            label="Linear model (Eq. 27)",
        )

    finite = df["RI"].dropna()
    if not finite.empty:
        peak_index = finite.abs().idxmax()
        peak_time = float(df.loc[peak_index, "time"])
        peak_value = float(df.loc[peak_index, "RI"])
        ax.plot(
            peak_time,
            peak_value,
            marker="o",
            ms=7,
            color=SIM_COLOR,
            mec=SURFACE,
            mew=1.6,
            zorder=4,
        )
        ax.annotate(
            f"peak {peak_value:+.2f} at {peak_time:.2f} s",
            xy=(peak_time, peak_value),
            xytext=(10, 8 if peak_value >= 0 else -14),
            textcoords="offset points",
            color=INK,
            fontsize=8.5,
        )

    lift_time = (summary or {}).get("wheel_lift_time_s")
    if lift_time is not None:
        for axis in axes:
            axis.axvline(lift_time, color=CRITICAL, lw=0.9, zorder=1)
        ax.annotate(
            f"{WHEEL_LABELS.get(summary['lifted_wheel'], summary['lifted_wheel'])} wheel lifts",
            xy=(lift_time, -1.0),
            xytext=(4, 6),
            textcoords="offset points",
            color=INK_SECONDARY,
            fontsize=8.5,
        )

    ri_values = pd.concat([df["RI"], df["RI_model"]]).dropna()
    extent = max(1.15, float(ri_values.abs().max()) * 1.05 if not ri_values.empty else 1.15)
    extent = min(extent, 3.0)
    ax.set_ylim(-extent, extent)
    _shade_ri_danger_zones(ax)
    ax.set_ylabel("RI (–)")
    _legend(ax)

    ax = axes[1]
    _shade_window(ax, summary)
    ax.plot(time, df["roll_deg"], color=SIM_COLOR)
    ax.set_title("Chassis roll angle (+ = rolls to the right, left side up)")
    ax.set_ylabel("Roll (°)")
    ax.set_xlabel("Time (s)")
    if summary is not None and summary.get("rolled_over"):
        ax.annotate(
            f"rolled over ({abs(df['roll_deg'].iloc[-1]):.0f}° roll)",
            xy=(time.iloc[-1], df["roll_deg"].iloc[-1]),
            xytext=(-8, 0),
            textcoords="offset points",
            ha="right",
            va="center",
            color=INK_SECONDARY,
            fontsize=8.5,
        )

    _window_note(fig, summary)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_tires(df: pd.DataFrame, summary: dict[str, Any] | None, out: Path):
    fig = plt.figure(figsize=(8.0, 9.0))
    grid = fig.add_gridspec(4, 1, height_ratios=[1.0, 1.0, 1.0, 0.55], hspace=0.45)
    fig.subplots_adjust(left=0.1, right=0.97, top=0.89, bottom=0.03)
    _title(fig, "Tire loads, lateral forces and slip angles", _conditions(summary, df))
    time = df["time"]
    weight = (
        summary["total_mass_kg"] * summary["vehicle_parameters"]["g_mps2"]
        if summary
        else None
    )

    axes = [fig.add_subplot(grid[0])]
    axes += [fig.add_subplot(grid[i], sharex=axes[0]) for i in (1, 2)]
    panels = (
        (
            "contact_Fz_N",
            "Vertical tire load / mg (paper normalization)" if weight else "Vertical tire load",
            "Fz / mg (–)" if weight else "Fz (N)",
            weight,
        ),
        ("Fy_wheel_N", "Lateral tire force (wheel heading frame, + = left)", "Fy (N)", None),
        ("slip_angle_deg", "Tire slip angle (+ = produces a left force)", "α (°)", None),
    )
    for ax, (suffix, title, label, scale) in zip(axes, panels):
        _shade_window(ax, summary)
        for wheel in WHEELS:
            values = df[f"{wheel}_{suffix}"]
            if scale:
                values = values / scale
            ax.plot(time, values, color=WHEEL_COLORS[wheel], label=WHEEL_LABELS[wheel])
        ax.set_title(title)
        ax.set_ylabel(label)
        if ax is not axes[-1]:
            plt.setp(ax.get_xticklabels(), visible=False)
    _legend(axes[0])
    axes[-1].set_xlabel("Time (s)")

    # Table view: the values the lines show, so no wheel is read by color alone.
    table_ax = fig.add_subplot(grid[3])
    table_ax.axis("off")
    steady = (summary or {}).get("steady_state")
    if steady:
        heading = "Steady-state mean ({:.1f}-{:.1f} s)".format(*steady["window_s"])
        rows = df[df["time"] >= steady["window_s"][0]]
    else:
        heading = f"Values at end of run (t = {time.iloc[-1]:.2f} s)"
        rows = df.tail(1)
    cells = []
    for wheel in WHEELS:
        fz = float(rows[f"{wheel}_contact_Fz_N"].mean())
        cells.append(
            [
                WHEEL_LABELS[wheel],
                f"{fz:,.0f}",
                f"{fz / weight:.3f}" if weight else "–",
                f"{float(rows[f'{wheel}_Fy_wheel_N'].mean()):,.0f}",
                f"{float(rows[f'{wheel}_slip_angle_deg'].mean()):.2f}",
            ]
        )
    table_height = 0.82
    table = table_ax.table(
        cellText=[["      " + row[0], *row[1:]] for row in cells],
        colLabels=["Wheel", "Fz (N)", "Fz / mg", "Fy (N)", "α (°)"],
        cellLoc="right",
        colLoc="right",
        loc="center",
        bbox=[0.0, 0.0, 1.0, table_height],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.set_linewidth(0.6)
        cell.set_facecolor(SURFACE)
        cell.get_text().set_color(INK if row else INK_SECONDARY)
        if col == 0:
            cell.get_text().set_ha("left")
            cell.PAD = 0.04
    # Color keys beside the wheel names; the table rows are equal height
    # inside the bbox (header row first).
    row_height = table_height / (len(WHEELS) + 1)
    for row, wheel in enumerate(WHEELS, start=1):
        table_ax.plot(
            [0.018],
            [table_height - (row + 0.5) * row_height],
            marker="s",
            ms=7,
            color=WHEEL_COLORS[wheel],
            transform=table_ax.transAxes,
            clip_on=False,
        )
    table_ax.set_title(heading, fontsize=9.5)

    fig.savefig(out, dpi=180)
    plt.close(fig)


def _fit_circle(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    design = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
    (xc, yc, c), *_ = np.linalg.lstsq(design, x * x + y * y, rcond=None)
    return float(xc), float(yc), float(math.sqrt(c + xc * xc + yc * yc))


def plot_trajectory(df: pd.DataFrame, summary: dict[str, Any] | None, out: Path):
    fig, ax = plt.subplots(figsize=(7.6, 7.6))
    fig.subplots_adjust(left=0.11, right=0.97, top=0.88, bottom=0.09)
    _title(fig, "Path of the vehicle center of gravity (plan view)", _conditions(summary, df))

    steady = (summary or {}).get("steady_state")
    if steady and steady.get("path_radius_m") is not None:
        window = df[df["time"] >= steady["window_s"][0]]
        xc, yc, radius = _fit_circle(window["CoG_x"].to_numpy(), window["CoG_y"].to_numpy())
        angles = np.linspace(0.0, 2.0 * math.pi, 361)
        ax.plot(
            xc + radius * np.cos(angles),
            yc + radius * np.sin(angles),
            color=MODEL_COLOR,
            lw=1.2,
            linestyle=(0, (5, 3)),
            label=f"Circle fit to steady state, R = {radius:.2f} m",
        )
        ax.plot(xc, yc, marker="+", ms=9, color=MUTED, mew=1.2)

    # Wheel-contact triangles once per second, like the paper's snapshot views.
    for t_mark in np.arange(0.0, df["time"].iloc[-1] + 1.0e-9, 1.0):
        row = df.iloc[(df["time"] - t_mark).abs().idxmin()]
        xs = [row[f"{w}_x"] for w in ("Wheel_F", "Wheel_RL", "Wheel_RR", "Wheel_F")]
        ys = [row[f"{w}_y"] for w in ("Wheel_F", "Wheel_RL", "Wheel_RR", "Wheel_F")]
        ax.fill(xs, ys, color=AXIS, alpha=0.35, lw=0)
        ax.plot(xs, ys, color=MUTED, lw=0.8)
    ax.plot([], [], color=MUTED, lw=0.8, label="Wheel-contact triangle, every 1 s")

    ax.plot(df["CoG_x"], df["CoG_y"], color=SIM_COLOR, label="CoG path")
    ax.plot(
        df["CoG_x"].iloc[0],
        df["CoG_y"].iloc[0],
        marker="o",
        ms=7,
        color=SIM_COLOR,
        mec=SURFACE,
        mew=1.6,
        zorder=4,
    )
    ax.annotate(
        "start",
        xy=(df["CoG_x"].iloc[0], df["CoG_y"].iloc[0]),
        xytext=(6, -12),
        textcoords="offset points",
        color=INK_SECONDARY,
        fontsize=8.5,
    )
    if summary is not None:
        onset = df.iloc[(df["time"] - summary["steering_start_time_s"]).abs().idxmin()]
        ax.plot(onset["CoG_x"], onset["CoG_y"], marker="o", ms=7, color=INK, mec=SURFACE, mew=1.6, zorder=4)
        ax.annotate(
            "steering starts",
            xy=(onset["CoG_x"], onset["CoG_y"]),
            xytext=(6, -12),
            textcoords="offset points",
            color=INK_SECONDARY,
            fontsize=8.5,
        )

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _legend(ax)
    fig.savefig(out, dpi=180)
    plt.close(fig)


# ----------------------------------------------------------------------
def print_comparison(summary: dict[str, Any], linear_ss: dict[str, float] | None):
    steady = summary.get("steady_state")
    print("\nConstant steer-angle cornering")
    print(f"  Steering angle:   {summary['steering_angle_deg']:.2f} deg")
    print(f"  Target speed:     {summary['target_speed_kph']:.1f} km/h")
    threshold = summary["rollover_threshold_ay_g"]
    print(
        "  Quasi-static rollover threshold (RI = 1): "
        f"{threshold['left_turn']:.3f} g left turn / {threshold['right_turn']:.3f} g right turn"
    )
    if summary.get("max_abs_RI") is not None:
        print(
            f"  Peak |RI|:        {summary['max_abs_RI']:.3f} at t = "
            f"{summary['max_abs_RI_time_s']:.2f} s"
        )
    if summary.get("wheel_lift_time_s") is not None:
        print(
            f"  Rear wheel lift:  {summary['lifted_wheel']} at t = "
            f"{summary['wheel_lift_time_s']:.3f} s (imminent rollover)"
        )
    if summary.get("rolled_over"):
        print(f"  ROLLED OVER at t = {summary['rollover_time_s']:.3f} s")
    if not steady:
        print("  No steady state (run ended early or too short).")
        return

    rows = [
        ("Yaw rate (deg/s)", steady["yaw_rate_dps"], "yaw_rate_dps", "{:.2f}"),
        ("Lateral acceleration (g)", steady["lateral_acceleration_g"], "lateral_acceleration_g", "{:.3f}"),
        ("Sideslip beta (deg)", steady["sideslip_deg"], "sideslip_deg", "{:.2f}"),
        ("Turn radius (m)", steady["turn_radius_m"], "turn_radius_m", "{:.2f}"),
        ("Rollover index RI (-)", steady["RI"], "RI", "{:.3f}"),
        (
            "Understeer gradient (deg/g)",
            steady["understeer_gradient_deg_per_g"],
            "understeer_gradient_deg_per_g",
            "{:+.3f}",
        ),
    ]
    print(f"\n  Steady state ({steady['window_s'][0]:.1f}-{steady['window_s'][1]:.1f} s)")
    print(f"  {'':30s}{'Simulation':>12s}{'Linear model':>14s}")
    for label, sim_value, key, fmt in rows:
        model = linear_ss.get(key) if linear_ss else None
        model_text = fmt.format(model) if model is not None else "-"
        sim_text = fmt.format(sim_value) if sim_value is not None else "-"
        print(f"  {label:30s}{sim_text:>12s}{model_text:>14s}")
    print(f"  {'RI estimate from a_y (Eq. 24)':30s}{steady['RI_model']:>12.3f}")
    print(f"  {'Roll angle (deg)':30s}{steady['roll_deg']:>12.2f}")


def main():
    parser = argparse.ArgumentParser(description="Plot a cornering telemetry CSV.")
    parser.add_argument(
        "csv",
        nargs="?",
        default=str(DEFAULT_CSV),
        help=f"Telemetry CSV written by cornering.py (default: {DEFAULT_CSV.name}).",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        csv_path = (ROOT / csv_path) if not csv_path.exists() else csv_path.resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing {csv_path}. Run cornering.py first.")

    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError(f"{csv_path} contains no telemetry.")

    summary_file = _summary_path(csv_path)
    summary = None
    if summary_file.exists():
        summary = json.loads(summary_file.read_text(encoding="utf-8"))
    else:
        print(f"Warning: {summary_file.name} not found; skipping the linear model.")

    linear = None
    linear_ss = None
    if summary is not None:
        params = summary["vehicle_parameters"]
        linear = linear_model_response(df, params)
        steady = summary.get("steady_state")
        if steady:
            linear_ss = linear_steady_state(
                params,
                steady["speed_kph"] / 3.6,
                math.radians(steady["steer_angle_deg"]),
            )

    _apply_style()
    prefix = _figure_prefix(csv_path)
    outputs = {
        "inputs": plot_inputs,
        "lateral_response": lambda d, s, o: plot_lateral_response(d, linear, s, o),
        "rollover": lambda d, s, o: plot_rollover(d, linear, s, o),
        "tires": plot_tires,
        "trajectory": plot_trajectory,
    }
    for name, plot in outputs.items():
        out = prefix.with_name(f"{prefix.name}_{name}.png")
        plot(df, summary, out)
        print(f"Plot written: {out.resolve()}")

    if summary is not None:
        print_comparison(summary, linear_ss)


if __name__ == "__main__":
    main()
