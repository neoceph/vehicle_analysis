from __future__ import annotations

import json
import math
from typing import Any

import pandas as pd

from ThreeWheeler import ThreeWheelerModel
from config import ROOT, load_json


def _call_method(instance: Any, method_name: str, *args: Any) -> Any:
    """Call a method on a PyChrono object without relying on its stubs."""
    method = getattr(instance, method_name, None)
    if not callable(method):
        raise AttributeError(
            f"{type(instance).__name__} exposes no {method_name}() method."
        )
    return method(*args)


def _component(value: Any, name: str) -> float:
    """Read a numeric vector or quaternion component across API versions."""
    component: Any = getattr(value, name)
    if callable(component):
        component = component()
    return float(component)


class BrakingSimulation:
    """Three-wheeler straight-line braking simulation.

    This replaces the Sedan-specific four-wheel topology in the original script
    with one front wheel and two rear wheels while preserving CSV telemetry.
    """

    def __init__(self):
        self.cfg = load_json("Braking_Config.json")
        self.model = ThreeWheelerModel()
        self.telemetry_data: list[dict[str, Any]] = []

        self.target_speed = float(self.cfg["Target_speed_kph"]) / 3.6
        self.step_size = float(self.cfg["Step_size_s"])
        self.max_time = float(self.cfg["Maximum_simulation_time_s"])
        self.output_hz = float(self.cfg["Output_frequency_hz"])
        self.settle_time = max(float(self.cfg.get("Initial_settle_time_s", 0.0)), 0.0)
        self.brake_start = float(self.cfg["Brake_start_time_s"])
        self.brake_ramp = max(float(self.cfg["Brake_ramp_time_s"]), 1e-9)
        self.stop_speed = float(self.cfg["Stop_speed_mps"])

        self.braking_start_time: float | None = None
        self.braking_start_x: float | None = None
        self.stop_time: float | None = None
        self.stop_x: float | None = None

        self._settle_model()
        self.model.set_initial_speed(self.target_speed)
        self.test_start_system_time = self.model.get_time()

    def _settle_model(self):
        """Resolve gravity and wheel-road contacts before the braking test."""
        road_friction = float(self.model.cfg["tire"]["Friction_coefficient"])
        self.model.set_contact_friction(0.0)
        self.model.set_brake_input(0.0)
        settle_steps = math.ceil(self.settle_time / self.step_size)
        for _ in range(settle_steps):
            self.model.advance(self.step_size)
        self.model.set_contact_friction(road_friction)

    def _test_time(self) -> float:
        return self.model.get_time() - self.test_start_system_time

    def _brake_input(self, time_s: float) -> float:
        if time_s < self.brake_start:
            return 0.0
        return min(1.0, (time_s - self.brake_start) / self.brake_ramp)

    def _log_frame(self, brake_input: float):
        self.model.compute_contact_forces()

        chassis = self.model.chassis
        pos = _call_method(chassis, "GetPos")
        rot = _call_method(chassis, "GetRot")
        speed = self.model.get_forward_speed()
        time_s = self._test_time()

        frame: dict[str, Any] = {
            "time": time_s,
            "speed_mps": speed,
            "speed_kph": speed * 3.6,
            "brake_input": brake_input,
            "Chassis_x": _component(pos, "x"),
            "Chassis_y": _component(pos, "y"),
            "Chassis_z": _component(pos, "z"),
            "Chassis_qw": _component(rot, "e0"),
            "Chassis_qx": _component(rot, "e1"),
            "Chassis_qy": _component(rot, "e2"),
            "Chassis_qz": _component(rot, "e3"),
        }

        for name, assembly in self.model.wheels.items():
            wheel = assembly.body
            w_pos = _call_method(wheel, "GetPos")
            w_rot = _call_method(wheel, "GetRot")
            w_ang = _call_method(wheel, "GetAngVelParent")
            road_force = self.model.get_wheel_road_force(name)

            try:
                brake_torque = float(
                    _call_method(
                        assembly.brake_motor,
                        "GetMotorTorque",
                    )
                )
            except Exception:
                brake_torque = 0.0

            frame.update(
                {
                    f"{name}_x": _component(w_pos, "x"),
                    f"{name}_y": _component(w_pos, "y"),
                    f"{name}_z": _component(w_pos, "z"),
                    f"{name}_qw": _component(w_rot, "e0"),
                    f"{name}_qx": _component(w_rot, "e1"),
                    f"{name}_qy": _component(w_rot, "e2"),
                    f"{name}_qz": _component(w_rot, "e3"),
                    f"{name}_omega_y": _component(w_ang, "y"),
                    f"{name}_brake_torque_Nm": brake_torque,
                    f"{name}_contact_Fx_N": road_force[0],
                    f"{name}_contact_Fy_N": road_force[1],
                    f"{name}_contact_Fz_N": road_force[2],
                    f"{name}_longitudinal_slip": (
                        assembly.last_longitudinal_slip
                    ),
                    f"{name}_lateral_slip": assembly.last_lateral_slip,
                    f"{name}_tire_deflection_m": (
                        assembly.last_tire_deflection_m
                    ),
                }
            )

        self.telemetry_data.append(frame)

    def run(self):
        last_sample_time = -1.0e9
        print("Starting three-wheeler headless braking simulation...")
        print(f"Stationary contact settling: {self.settle_time:.3f} s")
        print(f"Initial speed: {self.target_speed * 3.6:.1f} km/h")
        print(f"Total modeled mass: {self.model.total_mass():.1f} kg")

        while self._test_time() < self.max_time:
            time_s = self._test_time()
            speed = self.model.get_forward_speed()
            brake_input = self._brake_input(time_s)

            if time_s >= self.brake_start and self.braking_start_time is None:
                self.braking_start_time = time_s
                self.braking_start_x = _component(
                    _call_method(self.model.chassis, "GetPos"),
                    "x",
                )

            self.model.set_brake_input(brake_input)

            if time_s - last_sample_time >= (1.0 / self.output_hz) - (0.5 * self.step_size):
                self._log_frame(brake_input)
                last_sample_time = time_s

            if (
                self.braking_start_time is not None
                and time_s > self.brake_start + self.brake_ramp
                and abs(speed) <= self.stop_speed
            ):
                self.stop_time = time_s
                self.stop_x = _component(
                    _call_method(self.model.chassis, "GetPos"),
                    "x",
                )
                self._log_frame(brake_input)
                break

            self.model.advance(self.step_size)

        print("Simulation finished.")

    def export_results(self, output_path="results/simulation_telemetry.csv"):
        out_file = ROOT / output_path
        out_file.parent.mkdir(parents=True, exist_ok=True)

        df = pd.DataFrame(self.telemetry_data)
        df.to_csv(out_file, index=False)

        summary: dict[str, Any] = {
            "target_speed_kph": float(self.cfg["Target_speed_kph"]),
            "road_friction": float(self.model.cfg["tire"]["Friction_coefficient"]),
            "total_mass_kg": self.model.total_mass(),
            "braking_start_time_s": self.braking_start_time,
            "stop_time_s": self.stop_time,
            "braking_time_s": None,
            "braking_distance_m": None,
        }

        if (
            self.braking_start_time is not None
            and self.braking_start_x is not None
            and self.stop_time is not None
            and self.stop_x is not None
        ):
            summary["braking_time_s"] = self.stop_time - self.braking_start_time
            summary["braking_distance_m"] = self.stop_x - self.braking_start_x

        summary_file = out_file.parent / "braking_summary.json"
        summary_file.write_text(json.dumps(summary, indent=2), encoding="utf-8")

        print(f"Telemetry: {out_file.resolve()}")
        print(f"Summary:   {summary_file.resolve()}")
        if summary["braking_distance_m"] is not None:
            print(f"Braking distance: {summary['braking_distance_m']:.3f} m")
            print(f"Braking time:     {summary['braking_time_s']:.3f} s")


if __name__ == "__main__":
    sim = BrakingSimulation()
    sim.run()
    sim.export_results()
