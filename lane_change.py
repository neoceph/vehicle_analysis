from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Any

import pandas as pd

from config import load_json
from cornering import CorneringSimulation


@dataclass(frozen=True)
class SteeringProfile:
    """Prescribed front-wheel angles, in radians; positive steers left.

    One lane change uses two opposite sin-squared lobes. Reversing that
    pair after a dwell gives a double lane change. Angle and angle rate
    are continuous at every lobe boundary, including entry and exit.
    """

    amplitude_rad: float
    start_s: float
    duration_s: float
    double: bool = False
    dwell_s: float = 0.0

    def __post_init__(self):
        values = (self.amplitude_rad, self.start_s, self.duration_s, self.dwell_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Steering profile values must be finite.")
        if self.start_s < 0.0 or self.dwell_s < 0.0 or self.duration_s <= 0.0:
            raise ValueError("Start/dwell must be nonnegative and duration must be positive.")

    @property
    def return_start_s(self) -> float:
        return self.start_s + self.duration_s + self.dwell_s

    @property
    def end_s(self) -> float:
        if self.double:
            return self.return_start_s + self.duration_s
        return self.start_s + self.duration_s

    def _pair(self, elapsed_s: float) -> float:
        if elapsed_s <= 0.0 or elapsed_s >= self.duration_s:
            return 0.0
        fraction = elapsed_s / self.duration_s
        sign = 1.0 if fraction < 0.5 else -1.0
        return sign * self.amplitude_rad * math.sin(2.0 * math.pi * fraction) ** 2

    def angle(self, time_s: float) -> float:
        angle = self._pair(time_s - self.start_s)
        if self.double:
            angle -= self._pair(time_s - self.return_start_s)
        return angle

    def phase(self, time_s: float) -> str:
        if time_s < self.start_s:
            return "run_in"
        if time_s < self.start_s + self.duration_s:
            return "outbound"
        if self.double and time_s < self.return_start_s:
            return "dwell"
        if self.double and time_s < self.end_s:
            return "return"
        return "run_out"


class LaneChangeSimulation(CorneringSimulation):
    """Reuse the cornering vehicle, speed hold, telemetry, and RI definitions.

    The steering is open loop: no target path or lateral-position feedback
    modifies the prescribed angle. This follows the paper's steering-input
    validation approach, but does not digitize its Figure 4 waveform.
    """

    double = False
    config_file = "SingleLaneChange_Config.json"
    default_output = "results/single_lane_change_telemetry.csv"
    test_name = "single_lane_change"
    run_description = "single lane-change"

    def __init__(
        self,
        steering_angle_deg: float | None = None,
        target_speed_kph: float | None = None,
        duration_s: float | None = None,
        start_s: float | None = None,
        dwell_s: float | None = None,
        max_time_s: float | None = None,
    ):
        cfg = load_json(self.config_file)
        overrides = {
            key: float(value)
            for key, value in {
                "Steering_angle_deg": steering_angle_deg,
                "Target_speed_kph": target_speed_kph,
                "Lane_change_duration_s": duration_s,
                "Steering_start_time_s": start_s,
                "Between_lane_changes_time_s": dwell_s,
                "Maximum_simulation_time_s": max_time_s,
            }.items()
            if value is not None
        }
        cfg.update(overrides)
        self.profile = SteeringProfile(
            amplitude_rad=math.radians(float(cfg["Steering_angle_deg"])),
            start_s=float(cfg["Steering_start_time_s"]),
            duration_s=float(cfg["Lane_change_duration_s"]),
            double=self.double,
            dwell_s=float(cfg.get("Between_lane_changes_time_s", 0.0)),
        )
        for key in ("Target_speed_kph", "Maximum_simulation_time_s", "Step_size_s", "Output_frequency_hz"):
            if not math.isfinite(float(cfg[key])) or float(cfg[key]) <= 0.0:
                raise ValueError(f"{key} must be finite and positive.")
        sample_interval = max(float(cfg["Step_size_s"]), 1.0 / float(cfg["Output_frequency_hz"]))
        if float(cfg["Maximum_simulation_time_s"]) < self.profile.end_s + sample_interval:
            raise ValueError(
                f"Maximum_simulation_time_s must be at least "
                f"{self.profile.end_s + sample_interval:.3f} s to record the completed input."
            )
        wheel_cfg = load_json("ThreeWheeler_FrontWheel.json")
        limit = float(wheel_cfg.get("Max_steering_angle_deg", 45.0))
        if abs(float(cfg["Steering_angle_deg"])) > limit:
            raise ValueError(f"Steering amplitude exceeds the front-wheel limit of {limit:g} deg.")
        super().__init__(config_file=self.config_file, config_overrides=overrides)

    def _steering_input(self, time_s: float) -> float:
        return self.profile.angle(time_s)

    def _print_control_summary(self):
        print(f"Front-wheel steering amplitude: {math.degrees(self.profile.amplitude_rad):.2f} deg")
        print(
            f"Steer/countersteer duration per lane change: {self.profile.duration_s:.2f} s; "
            f"input starts at {self.profile.start_s:.2f} s and ends at {self.profile.end_s:.2f} s"
        )
        if self.double:
            print(f"Dwell between lane changes: {self.profile.dwell_s:.2f} s")

    def _log_frame(self, steering_command: float):
        super()._log_frame(steering_command)
        frame = self.telemetry_data[-1]
        first = self.telemetry_data[0]
        frame.update(
            {
                "maneuver_phase": self.profile.phase(float(frame["time"])),
                "longitudinal_displacement_m": frame["CoG_x"] - first["CoG_x"],
                "lateral_displacement_m": frame["CoG_y"] - first["CoG_y"],
                "heading_change_deg": frame["yaw_deg"] - first["yaw_deg"],
            }
        )

    def _steady_state(self, df: pd.DataFrame) -> None:
        # A transient lane change has no constant-turn steady state.
        return None

    def _extra_summary_fields(self, df: pd.DataFrame) -> dict[str, Any]:
        end_time = float(df["time"].iloc[-1])
        completed = end_time >= self.profile.end_s
        return {
            "steering_ramp_time_s": None,
            "steering_input": {
                "type": "prescribed_front_wheel_angle",
                "shape": "opposite_sin_squared_lobes",
                "amplitude_deg": math.degrees(self.profile.amplitude_rad),
                "start_time_s": self.profile.start_s,
                "duration_per_lane_change_s": self.profile.duration_s,
                "number_of_lane_changes": 2 if self.double else 1,
                "dwell_time_s": self.profile.dwell_s if self.double else 0.0,
                "return_start_time_s": self.profile.return_start_s if self.double else None,
                "end_time_s": self.profile.end_s,
            },
            "maneuver_completed": completed,
            "final_lateral_displacement_m": float(df["lateral_displacement_m"].iloc[-1]),
            "max_abs_lateral_displacement_m": float(df["lateral_displacement_m"].abs().max()),
            "final_heading_change_deg": float(df["heading_change_deg"].iloc[-1]),
            "max_abs_yaw_rate_dps": float(df["yaw_rate_dps"].abs().max()),
            "max_abs_sideslip_deg": float(df["beta_deg"].abs().max()),
            "max_abs_steer_angle_deg": float(df["steer_angle_deg"].abs().max()),
        }

    def export_results(self, output_path: str | None = None):
        super().export_results(output_path or self.default_output)


def run_cli(simulation_class: type[LaneChangeSimulation]):
    parser = argparse.ArgumentParser(
        description=(
            f"Three-wheeler {simulation_class.run_description} with prescribed front-wheel angles. "
            f"Defaults: {simulation_class.config_file}."
        )
    )
    parser.add_argument("--steering-deg", type=float, help="Signed peak front-wheel angle in degrees (+ = left first).")
    parser.add_argument("--speed-kph", type=float, help="Target speed in km/h.")
    parser.add_argument("--duration-s", type=float, help="Duration of one steer/countersteer pair in seconds.")
    parser.add_argument("--start-s", type=float, help="Input start time after settling, in seconds.")
    if simulation_class.double:
        parser.add_argument("--dwell-s", type=float, help="Straight dwell between the outbound and return pairs.")
    parser.add_argument("--max-time-s", type=float, help="Total simulation duration after settling, in seconds.")
    parser.add_argument("--output", default=simulation_class.default_output, help="Telemetry CSV path (relative to this folder or absolute).")
    args = parser.parse_args()
    sim = simulation_class(
        steering_angle_deg=args.steering_deg,
        target_speed_kph=args.speed_kph,
        duration_s=args.duration_s,
        start_s=args.start_s,
        dwell_s=getattr(args, "dwell_s", None),
        max_time_s=args.max_time_s,
    )
    sim.run()
    sim.export_results(args.output)
