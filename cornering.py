from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from ThreeWheeler import ThreeWheelerModel
from config import ROOT, load_json


CONFIG_FILE = "Cornering_Config.json"
DEFAULT_OUTPUT = "results/cornering_constant_steer_telemetry.csv"
REAR_WHEELS = ("Wheel_RL", "Wheel_RR")


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


def _xyz(value: Any) -> np.ndarray:
    return np.array([_component(value, axis) for axis in ("x", "y", "z")])


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _roll_pitch_yaw(rot: Any) -> tuple[float, float, float]:
    """Z-Y-X (yaw-pitch-roll) Euler angles of a Chrono quaternion, in rad.

    Roll is positive when the left side rises (body rolls to the right).
    """
    w = _component(rot, "e0")
    x = _component(rot, "e1")
    y = _component(rot, "e2")
    z = _component(rot, "e3")
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _json_safe(value: Any) -> Any:
    """Replace NaN/inf (not valid JSON) with None, recursively."""
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _summary_path(csv_path: Path) -> Path:
    """Sibling summary JSON of a telemetry CSV (…_telemetry.csv -> …_summary.json)."""
    stem = csv_path.stem
    if stem.endswith("_telemetry"):
        stem = stem[: -len("_telemetry")]
    return csv_path.with_name(f"{stem}_summary.json")


class CorneringSimulation:
    """Three-wheeler constant steer-angle cornering simulation.

    After stationary settling and a straight run-in at the target speed,
    the front wheel is steered to a constant angle and held while a PI
    controller drives the rear wheels to hold the speed, as assumed in
    Rodriguez Licea et al. (2018), "The Rollover Risk in Delta Tricycles".

    Rollover index (RI), following the paper's text and figures:

        RI = (Fz_RR - Fz_RL) / (Fz_RR + Fz_RL)

    +1 means the left rear wheel is unloaded (rollover to the right, the
    outside of a left turn); -1 means the right rear wheel is unloaded.
    Eq. (17) as printed has the numerator reversed relative to that text.
    RI_model is the paper's accelerometer estimate, Eq. (24), with the same
    sign convention and a_y positive to the left (ISO 8855):

        RI_model = 2 h l a_y / (b g l_f) - l (b_r - b_l) / (b l_f)
    """

    def __init__(
        self,
        steering_angle_deg: float | None = None,
        target_speed_kph: float | None = None,
    ):
        self.cfg = load_json(CONFIG_FILE)
        if steering_angle_deg is not None:
            self.cfg["Steering_angle_deg"] = float(steering_angle_deg)
        if target_speed_kph is not None:
            self.cfg["Target_speed_kph"] = float(target_speed_kph)

        self.model = ThreeWheelerModel(
            enable_steering=True,
            scenario_config=CONFIG_FILE,
        )
        self.telemetry_data: list[dict[str, Any]] = []

        self.target_speed = float(self.cfg["Target_speed_kph"]) / 3.6
        self.steering_angle = math.radians(float(self.cfg["Steering_angle_deg"]))
        self.steering_start = float(self.cfg["Steering_start_time_s"])
        self.steering_ramp = max(float(self.cfg["Steering_ramp_time_s"]), 1e-9)
        self.step_size = float(self.cfg["Step_size_s"])
        self.max_time = float(self.cfg["Maximum_simulation_time_s"])
        self.output_hz = float(self.cfg["Output_frequency_hz"])
        self.settle_time = max(float(self.cfg.get("Initial_settle_time_s", 0.0)), 0.0)
        self.gravity = float(self.cfg["Gravity_mps2"])

        self.drive_wheels = tuple(self.cfg["Drive_wheels"])
        self.speed_kp = float(self.cfg["Speed_kp_Nm_per_mps"])
        self.speed_ki = float(self.cfg["Speed_ki_Nm_per_m"])
        self.max_drive_torque = float(self.cfg["Max_drive_torque_Nm"])
        self.lift_force = float(self.cfg["Wheel_lift_force_N"])
        self.rollover_roll = math.radians(float(self.cfg["Rollover_roll_angle_deg"]))

        self.drive_torque = 0.0
        self.speed_integral = 0.0
        self.wheel_lift_time: float | None = None
        self.lifted_wheel: str | None = None
        self.wheel_lift_duration = 0.0
        self.rollover_time: float | None = None
        self.max_abs_roll = 0.0

        self._settle_model()
        self.model.set_initial_speed(self.target_speed)
        self.test_start_system_time = self.model.get_time()
        self.mass = self.model.total_mass()
        self.params = self._vehicle_parameters()
        self.prev_yaw = _roll_pitch_yaw(_call_method(self.model.chassis, "GetRot"))[2]
        self.yaw_unwrapped = self.prev_yaw

        # Start the integrator at the straight-line rolling resistance
        # torque so the run-in starts close to steady state.
        rolling_torque = (
            float(self.model.cfg["tire"].get("Rolling_resistance_coefficient", 0.0))
            * self.mass
            * self.gravity
            * float(self.model.cfg["rear_wheel"]["Radius_m"])
        )
        if self.speed_ki > 0.0:
            self.speed_integral = rolling_torque / self.speed_ki

    def _settle_model(self):
        """Resolve gravity and wheel-road contacts before the cornering test."""
        road_friction = float(self.model.cfg["tire"]["Friction_coefficient"])
        self.model.set_contact_friction(0.0)
        self.model.set_brake_input(0.0)
        self.model.set_steering_angle(0.0)
        settle_steps = math.ceil(self.settle_time / self.step_size)
        for _ in range(settle_steps):
            self.model.advance(self.step_size)
        self.model.set_contact_friction(road_friction)

    def _test_time(self) -> float:
        return self.model.get_time() - self.test_start_system_time

    def _steering_input(self, time_s: float) -> float:
        """Constant steer angle reached through a raised-cosine ramp."""
        if time_s <= self.steering_start:
            return 0.0
        progress = min(1.0, (time_s - self.steering_start) / self.steering_ramp)
        return self.steering_angle * 0.5 * (1.0 - math.cos(math.pi * progress))

    def _speed_control(self) -> float:
        """PI drive torque that holds the target forward speed."""
        error = self.target_speed - self.model.get_forward_speed()
        integral = self.speed_integral + error * self.step_size
        torque = self.speed_kp * error + self.speed_ki * integral
        limited = max(-self.max_drive_torque, min(self.max_drive_torque, torque))
        # Conditional integration: hold the integrator while saturated.
        if limited == torque:
            self.speed_integral = integral
        self.drive_torque = limited
        return limited

    def _heading_axes(self, yaw: float) -> tuple[np.ndarray, np.ndarray]:
        forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        return forward, left

    def _cog_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Position and velocity of the whole-vehicle center of gravity."""
        position = np.zeros(3)
        velocity = np.zeros(3)
        for body in self.model.vehicle_bodies():
            mass = float(_call_method(body, "GetMass"))
            position += mass * _xyz(_call_method(body, "GetPos"))
            velocity += mass * _xyz(_call_method(body, "GetPosDt"))
        return position / self.mass, velocity / self.mass

    def _vehicle_parameters(self) -> dict[str, float]:
        """Lateral-model parameters in the paper's notation (cf. Table 1).

        Geometry is measured on the settled vehicle; the static wheel loads
        solve the vertical, roll, and pitch balance of the rigid vehicle;
        cornering stiffnesses are the TMeasy small-slip slopes at those loads.
        """
        _, _, yaw = _roll_pitch_yaw(_call_method(self.model.chassis, "GetRot"))
        forward, left = self._heading_axes(yaw)
        cog, _ = self._cog_state()

        def wheel_position(name: str) -> np.ndarray:
            return _xyz(_call_method(self.model.wheels[name].body, "GetPos"))

        front = wheel_position("Wheel_F")
        rear_left = wheel_position("Wheel_RL")
        rear_right = wheel_position("Wheel_RR")
        rear_mid = 0.5 * (rear_left + rear_right)

        wheelbase = float(np.dot(front - rear_mid, forward))
        l_r = float(np.dot(cog - rear_mid, forward))
        l_f = wheelbase - l_r
        b_l = float(np.dot(rear_left - cog, left))
        b_r = float(np.dot(cog - rear_right, left))
        track = b_l + b_r
        height = float(cog[2])

        yaw_inertia = 0.0
        for body in self.model.vehicle_bodies():
            mass = float(_call_method(body, "GetMass"))
            offset = _xyz(_call_method(body, "GetPos")) - cog
            yaw_inertia += _component(_call_method(body, "GetInertiaXX"), "z")
            yaw_inertia += mass * float(offset[0] ** 2 + offset[1] ** 2)

        # Static loads: total weight, then zero roll and pitch moments about
        # the CoG for wheel contacts directly below the wheel centers.
        contacts = [front, rear_left, rear_right]
        equilibrium = np.array(
            [
                [1.0, 1.0, 1.0],
                [float(np.dot(p - cog, left)) for p in contacts],
                [float(np.dot(p - cog, forward)) for p in contacts],
            ]
        )
        weight = self.mass * self.gravity
        fz_f, fz_rl, fz_rr = np.linalg.solve(
            equilibrium,
            np.array([weight, 0.0, 0.0]),
        )

        c_f = self.model.tire_cornering_stiffness(fz_f)
        c_r = (
            self.model.tire_cornering_stiffness(fz_rl)
            + self.model.tire_cornering_stiffness(fz_rr)
        )

        return {
            "m_kg": self.mass,
            "J_kg_m2": yaw_inertia,
            "l_m": wheelbase,
            "l_f_m": l_f,
            "l_r_m": l_r,
            "h_m": height,
            "b_m": track,
            "b_l_m": b_l,
            "b_r_m": b_r,
            "g_mps2": self.gravity,
            "c_f_N_per_rad": c_f,
            "c_r_N_per_rad": c_r,
            "static_Fz_F_N": float(fz_f),
            "static_Fz_RL_N": float(fz_rl),
            "static_Fz_RR_N": float(fz_rr),
        }

    def _rollover_index_model(self, ay: float) -> float:
        """Paper Eq. (24) with the RI sign convention of this class."""
        p = self.params
        gain = 2.0 * p["h_m"] * p["l_m"] / (p["b_m"] * p["g_mps2"] * p["l_f_m"])
        offset = p["l_m"] * (p["b_r_m"] - p["b_l_m"]) / (p["b_m"] * p["l_f_m"])
        return gain * ay - offset

    def _track_events(self, time_s: float):
        """Check every step for rear-wheel lift and rollover."""
        roll, _, _ = _roll_pitch_yaw(_call_method(self.model.chassis, "GetRot"))
        self.max_abs_roll = max(self.max_abs_roll, abs(roll))

        lifted = [
            name
            for name in REAR_WHEELS
            if self.model.wheels[name].last_normal_force_N <= self.lift_force
        ]
        if lifted:
            self.wheel_lift_duration += self.step_size
            if self.wheel_lift_time is None:
                self.wheel_lift_time = time_s
                self.lifted_wheel = lifted[0]

        if abs(roll) >= self.rollover_roll and self.rollover_time is None:
            self.rollover_time = time_s

    def _log_frame(self, steering_command: float):
        self.model.compute_contact_forces()

        chassis = self.model.chassis
        pos = _call_method(chassis, "GetPos")
        rot = _call_method(chassis, "GetRot")
        roll, pitch, yaw = _roll_pitch_yaw(rot)
        self.yaw_unwrapped += _wrap_angle(yaw - self.prev_yaw)
        self.prev_yaw = yaw
        forward, left = self._heading_axes(yaw)

        cog, cog_velocity = self._cog_state()
        vx = float(np.dot(cog_velocity, forward))
        vy = float(np.dot(cog_velocity, left))
        speed = self.model.get_forward_speed()

        # Tire forces are the only horizontal external forces on the vehicle,
        # so they give the exact CoG acceleration (paper Eq. 25).
        road_forces = {
            name: np.array(self.model.get_wheel_road_force(name))
            for name in self.model.wheels
        }
        acceleration = sum(road_forces.values()) / self.mass
        ax = float(np.dot(acceleration, forward))
        ay = float(np.dot(acceleration, left))

        fz_rl = float(road_forces["Wheel_RL"][2])
        fz_rr = float(road_forces["Wheel_RR"][2])
        rear_load = fz_rl + fz_rr
        rollover_index = (fz_rr - fz_rl) / rear_load if rear_load > 1.0e-6 else math.nan

        chassis_omega = _call_method(chassis, "GetAngVelParent")
        chassis_omega_local = _call_method(chassis, "GetAngVelLocal")

        frame: dict[str, Any] = {
            "time": self._test_time(),
            "speed_mps": speed,
            "speed_kph": speed * 3.6,
            "steer_cmd_deg": math.degrees(steering_command),
            "steer_angle_deg": math.degrees(self.model.get_steering_angle()),
            "steer_torque_Nm": self.model.get_steering_torque(),
            "drive_torque_Nm": self.drive_torque,
            "v_cog_mps": math.hypot(vx, vy),
            "vx_mps": vx,
            "vy_mps": vy,
            "beta_deg": math.degrees(math.atan2(vy, vx)) if vx > 0.1 else 0.0,
            "yaw_deg": math.degrees(self.yaw_unwrapped),
            "yaw_rate_dps": math.degrees(_component(chassis_omega, "z")),
            "roll_deg": math.degrees(roll),
            "roll_rate_dps": math.degrees(_component(chassis_omega_local, "x")),
            "pitch_deg": math.degrees(pitch),
            "ax_mps2": ax,
            "ay_mps2": ay,
            "ay_g": ay / self.gravity,
            "RI": rollover_index,
            "RI_model": self._rollover_index_model(ay),
            "CoG_x": float(cog[0]),
            "CoG_y": float(cog[1]),
            "CoG_z": float(cog[2]),
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
            w_ang = _call_method(wheel, "GetAngVelLocal")
            road_force = road_forces[name]
            fx_wheel, fy_wheel, _ = assembly.last_tire_force_wheel_N

            frame.update(
                {
                    f"{name}_x": _component(w_pos, "x"),
                    f"{name}_y": _component(w_pos, "y"),
                    f"{name}_z": _component(w_pos, "z"),
                    f"{name}_qw": _component(w_rot, "e0"),
                    f"{name}_qx": _component(w_rot, "e1"),
                    f"{name}_qy": _component(w_rot, "e2"),
                    f"{name}_qz": _component(w_rot, "e3"),
                    # Spin rate about the wheel axle.
                    f"{name}_omega_y": _component(w_ang, "y"),
                    f"{name}_drive_torque_Nm": assembly.drive_torque_Nm,
                    f"{name}_brake_torque_Nm": assembly.brake_torque_Nm,
                    f"{name}_contact_Fx_N": float(road_force[0]),
                    f"{name}_contact_Fy_N": float(road_force[1]),
                    f"{name}_contact_Fz_N": float(road_force[2]),
                    f"{name}_Fx_wheel_N": fx_wheel,
                    f"{name}_Fy_wheel_N": fy_wheel,
                    f"{name}_slip_angle_deg": math.degrees(
                        assembly.last_slip_angle_rad
                    ),
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
        print("Starting three-wheeler headless constant steer-angle simulation...")
        print(f"Stationary contact settling: {self.settle_time:.3f} s")
        print(f"Target speed:   {self.target_speed * 3.6:.1f} km/h")
        print(
            f"Steering angle: {math.degrees(self.steering_angle):.2f} deg "
            f"(front wheel, + = left) from t = {self.steering_start:.2f} s"
        )
        print(f"Total modeled mass: {self.mass:.1f} kg")

        while self._test_time() < self.max_time:
            time_s = self._test_time()
            steering_command = self._steering_input(time_s)
            self.model.set_steering_angle(steering_command)
            self.model.set_drive_torque(self._speed_control(), self.drive_wheels)
            self._track_events(time_s)

            sample_due = (
                time_s - last_sample_time
                >= (1.0 / self.output_hz) - (0.5 * self.step_size)
            )
            if sample_due or self.rollover_time is not None:
                self._log_frame(steering_command)
                last_sample_time = time_s

            if self.rollover_time is not None:
                print(f"Rollover detected at t = {self.rollover_time:.3f} s.")
                break

            self.model.advance(self.step_size)

        print("Simulation finished.")

    def _steady_state(self, df: pd.DataFrame) -> dict[str, Any] | None:
        """Average the last Steady_state_window_s of a completed run."""
        window = float(self.cfg["Steady_state_window_s"])
        end_time = float(df["time"].iloc[-1])
        ramp_end = self.steering_start + self.steering_ramp
        if self.rollover_time is not None or end_time - window < ramp_end:
            return None

        steady = df[df["time"] >= end_time - window]
        speed = float(steady["v_cog_mps"].mean())
        yaw_rate = math.radians(float(steady["yaw_rate_dps"].mean()))
        ay = float(steady["ay_mps2"].mean())
        steer = math.radians(float(steady["steer_angle_deg"].mean()))

        # Algebraic (Kasa) circle fit of the CoG path; a short arc (nearly
        # straight path) leaves the circle undetermined.
        path_radius = None
        heading_change = abs(float(steady["yaw_deg"].iloc[-1] - steady["yaw_deg"].iloc[0]))
        if heading_change >= 5.0:
            x = steady["CoG_x"].to_numpy()
            y = steady["CoG_y"].to_numpy()
            design = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
            (xc, yc, c), *_ = np.linalg.lstsq(design, x * x + y * y, rcond=None)
            path_radius = float(math.sqrt(c + xc * xc + yc * yc))

        yaw_rate_std = math.radians(float(steady["yaw_rate_dps"].std()))
        reached = (
            yaw_rate_std <= max(0.02 * abs(yaw_rate), math.radians(0.1))
            and abs(speed - self.target_speed) <= 0.02 * self.target_speed
        )

        radius = abs(speed / yaw_rate) if abs(yaw_rate) > 1.0e-6 else None

        # Bicycle-model equivalent understeer gradient: delta - l/R is
        # alpha_f - alpha_r = K a_y.  Using the slip angles avoids the
        # small-angle error of l/R, which is comparable to K for a
        # near-neutral vehicle on a tight circle.
        alpha_front = math.radians(float(steady["Wheel_F_slip_angle_deg"].mean()))
        alpha_rear = math.radians(
            0.5
            * float(
                steady["Wheel_RL_slip_angle_deg"].mean()
                + steady["Wheel_RR_slip_angle_deg"].mean()
            )
        )
        understeer = (alpha_front - alpha_rear) / ay if abs(ay) > 1.0e-6 else None

        result: dict[str, Any] = {
            "window_s": [float(steady["time"].iloc[0]), end_time],
            "reached": bool(reached),
            "speed_kph": speed * 3.6,
            "steer_angle_deg": math.degrees(steer),
            "yaw_rate_dps": math.degrees(yaw_rate),
            "yaw_rate_std_dps": math.degrees(yaw_rate_std),
            "lateral_acceleration_mps2": ay,
            "lateral_acceleration_g": ay / self.gravity,
            "sideslip_deg": float(steady["beta_deg"].mean()),
            "roll_deg": float(steady["roll_deg"].mean()),
            "turn_radius_m": radius,
            "path_radius_m": path_radius,
            "understeer_gradient_deg_per_g": (
                math.degrees(understeer) * self.gravity
                if understeer is not None
                else None
            ),
            "RI": float(steady["RI"].mean()),
            "RI_model": float(steady["RI_model"].mean()),
            "drive_torque_Nm": float(steady["drive_torque_Nm"].mean()),
        }
        for name in self.model.wheels:
            result[f"{name}_Fz_N"] = float(steady[f"{name}_contact_Fz_N"].mean())
            result[f"{name}_Fy_wheel_N"] = float(steady[f"{name}_Fy_wheel_N"].mean())
            result[f"{name}_slip_angle_deg"] = float(
                steady[f"{name}_slip_angle_deg"].mean()
            )
        return result

    def export_results(self, output_path=DEFAULT_OUTPUT):
        out_file = ROOT / output_path
        out_file.parent.mkdir(parents=True, exist_ok=True)

        df = pd.DataFrame(self.telemetry_data)
        df.to_csv(out_file, index=False)

        p = self.params
        ri_gain = 2.0 * p["h_m"] * p["l_m"] / (p["b_m"] * p["g_mps2"] * p["l_f_m"])
        ri_offset = p["l_m"] * (p["b_r_m"] - p["b_l_m"]) / (p["b_m"] * p["l_f_m"])
        understeer_gradient = (p["m_kg"] / p["l_m"]) * (
            p["l_r_m"] / p["c_f_N_per_rad"] - p["l_f_m"] / p["c_r_N_per_rad"]
        )

        ri = df["RI"].abs()
        peak_row = int(ri.idxmax()) if ri.notna().any() else None
        summary: dict[str, Any] = {
            "test": "constant_steer_angle",
            "target_speed_kph": float(self.cfg["Target_speed_kph"]),
            "steering_angle_deg": float(self.cfg["Steering_angle_deg"]),
            "steering_start_time_s": self.steering_start,
            "steering_ramp_time_s": self.steering_ramp,
            "road_friction": float(self.model.cfg["tire"]["Friction_coefficient"]),
            "total_mass_kg": self.mass,
            "drive_wheels": list(self.drive_wheels),
            "end_time_s": float(df["time"].iloc[-1]),
            "vehicle_parameters": p,
            "rollover_threshold_ay_g": {
                # Quasi-static lateral acceleration at which RI_model = +/-1.
                "left_turn": (1.0 + ri_offset) / ri_gain / p["g_mps2"],
                "right_turn": (1.0 - ri_offset) / ri_gain / p["g_mps2"],
            },
            "linear_understeer_gradient_deg_per_g": (
                math.degrees(understeer_gradient) * p["g_mps2"]
            ),
            "wheel_lift_time_s": self.wheel_lift_time,
            "lifted_wheel": self.lifted_wheel,
            "wheel_lift_duration_s": self.wheel_lift_duration,
            "rolled_over": self.rollover_time is not None,
            "rollover_time_s": self.rollover_time,
            "max_abs_roll_deg": math.degrees(self.max_abs_roll),
            "max_abs_RI": float(ri.max()) if peak_row is not None else None,
            "max_abs_RI_time_s": (
                float(df["time"].iloc[peak_row]) if peak_row is not None else None
            ),
            "max_abs_ay_g": float(df["ay_g"].abs().max()),
            "steady_state": self._steady_state(df),
        }

        summary_file = _summary_path(out_file)
        summary_file.write_text(
            json.dumps(_json_safe(summary), indent=2),
            encoding="utf-8",
        )

        print(f"Telemetry: {out_file.resolve()}")
        print(f"Summary:   {summary_file.resolve()}")
        steady = summary["steady_state"]
        if steady is not None:
            print(f"Steady-state yaw rate:      {steady['yaw_rate_dps']:.2f} deg/s")
            print(f"Steady-state lateral accel: {steady['lateral_acceleration_g']:.3f} g")
            if steady["turn_radius_m"] is not None:
                print(f"Steady-state turn radius:   {steady['turn_radius_m']:.2f} m")
            print(f"Steady-state RI:            {steady['RI']:.3f}")
        if self.wheel_lift_time is not None:
            print(
                f"Rear wheel lift ({self.lifted_wheel}) at "
                f"t = {self.wheel_lift_time:.3f} s: imminent rollover."
            )
        if summary["rolled_over"]:
            print(f"Vehicle rolled over at t = {self.rollover_time:.3f} s.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Constant steer-angle cornering test of the three-wheeler. "
            f"Defaults come from {CONFIG_FILE}."
        )
    )
    parser.add_argument(
        "--steering-deg",
        type=float,
        help="Constant front-wheel steer angle in degrees (+ = left turn).",
    )
    parser.add_argument(
        "--speed-kph",
        type=float,
        help="Target vehicle speed in km/h.",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help=f"Telemetry CSV path relative to this folder (default: {DEFAULT_OUTPUT}).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    sim = CorneringSimulation(
        steering_angle_deg=args.steering_deg,
        target_speed_kph=args.speed_kph,
    )
    sim.run()
    sim.export_results(args.output)
