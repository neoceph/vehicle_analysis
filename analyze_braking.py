from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent
CSV = ROOT / "results" / "simulation_telemetry.csv"
OUT = ROOT / "results"


def main():
    if not CSV.exists():
        raise FileNotFoundError(f"Missing {CSV}. Run braking.py first.")

    df = pd.read_csv(CSV)
    braking = df[df["brake_input"] > 0].copy()
    if braking.empty:
        raise RuntimeError("No braking phase was found in the telemetry.")

    t0 = braking["time"].iloc[0]
    x0 = braking["Chassis_x"].iloc[0]
    braking["braking_time"] = braking["time"] - t0
    braking["braking_dist"] = braking["Chassis_x"] - x0

    plt.figure()
    plt.plot(braking["braking_time"], braking["speed_kph"])
    plt.xlabel("Braking time [s]")
    plt.ylabel("Vehicle speed [km/h]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUT / "speed_vs_time.png", dpi=200)
    plt.close()

    plt.figure()
    plt.plot(braking["braking_dist"], braking["speed_kph"])
    plt.xlabel("Braking distance [m]")
    plt.ylabel("Vehicle speed [km/h]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUT / "speed_vs_distance.png", dpi=200)
    plt.close()

    plt.figure()
    for wheel in ("Wheel_F", "Wheel_RL", "Wheel_RR"):
        plt.plot(braking["braking_time"], braking[f"{wheel}_contact_Fz_N"], label=wheel)
    plt.xlabel("Braking time [s]")
    plt.ylabel("Vertical tire contact force [N]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUT / "normal_loads_vs_time.png", dpi=200)
    plt.close()

    plt.figure()
    for wheel in ("Wheel_F", "Wheel_RL", "Wheel_RR"):
        plt.plot(braking["braking_time"], braking[f"{wheel}_omega_y"], label=wheel)
    plt.xlabel("Braking time [s]")
    plt.ylabel("Wheel angular velocity about Y [rad/s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUT / "wheel_speed_vs_time.png", dpi=200)
    plt.close()

    print(f"Plots written to {OUT.resolve()}")


if __name__ == "__main__":
    main()
