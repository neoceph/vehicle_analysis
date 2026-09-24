from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict

try:
    import pychrono.core as chrono
except ImportError:
    import pychrono as chrono

from config import load_all


def _chrono_vector(x: float, y: float, z: float):
    vector_cls = getattr(
        chrono,
        "ChVector3d",
        getattr(chrono, "ChVectorD", None),
    )
    if vector_cls is None:
        raise AttributeError(
            "This PyChrono build exposes neither ChVector3d nor ChVectorD."
        )
    return vector_cls(float(x), float(y), float(z))


def _chrono_frame(position, rotation=None):
    frame_cls = getattr(
        chrono,
        "ChFramed",
        getattr(
            chrono,
            "ChFrameD",
            getattr(chrono, "ChFrame", None),
        ),
    )
    if frame_cls is None:
        raise AttributeError(
            "This PyChrono build exposes neither ChFramed, ChFrameD, nor ChFrame."
        )
    if rotation is None:
        return frame_cls(position)
    return frame_cls(position, rotation)


def _chrono_axis_y():
    axis_y = getattr(chrono, "ChAxis_Y", None)
    if axis_y is not None:
        return axis_y

    axis_enum = getattr(chrono, "ChAxis", None)
    if axis_enum is not None:
        axis_y = getattr(axis_enum, "Y", None)
        if axis_y is not None:
            return axis_y

    raise AttributeError(
        "This PyChrono build exposes neither ChAxis_Y nor ChAxis.Y."
    )


def _chrono_easy_cylinder(radius, width, density, visualize, collide, material):
    try:
        return chrono.ChBodyEasyCylinder(
            _chrono_axis_y(),
            float(radius),
            float(width),
            float(density),
            bool(visualize),
            bool(collide),
            material,
        )
    except (AttributeError, TypeError):
        # Older PyChrono bindings used body Y as the cylinder axis by default.
        return chrono.ChBodyEasyCylinder(
            float(radius),
            float(width),
            float(density),
            bool(visualize),
            bool(collide),
            material,
        )


def _chrono_rotate_z_to_y():
    rotation = getattr(chrono, "Q_ROTATE_Z_TO_Y", None)
    if rotation is not None:
        return rotation

    quat_from_angle_x = getattr(chrono, "QuatFromAngleX", None)
    if quat_from_angle_x is not None:
        return quat_from_angle_x(-math.pi / 2.0)

    raise AttributeError(
        "This PyChrono build exposes neither Q_ROTATE_Z_TO_Y nor QuatFromAngleX()."
    )


def _chrono_const_function(value: float):
    function_cls = getattr(
        chrono,
        "ChFunctionConst",
        getattr(chrono, "ChFunction_Const", None),
    )
    if function_cls is None:
        raise AttributeError(
            "This PyChrono build exposes neither ChFunctionConst nor ChFunction_Const."
        )
    return function_cls(float(value))


def _chrono_contact_material_smc():
    material_cls = getattr(
        chrono,
        "ChContactMaterialSMC",
        getattr(chrono, "ChMaterialSurfaceSMC", None),
    )
    if material_cls is None:
        raise AttributeError(
            "This PyChrono build exposes neither ChContactMaterialSMC "
            "nor ChMaterialSurfaceSMC."
        )
    return material_cls()


def _call_first_method(
    instance: Any,
    method_names: tuple[str, ...],
    *args: Any,
) -> Any:
    """Call the first available method from a version-dependent API."""
    for method_name in method_names:
        method = getattr(instance, method_name, None)
        if callable(method):
            return method(*args)

    joined_names = " or ".join(
        f"{method_name}()" for method_name in method_names
    )
    raise AttributeError(
        f"{type(instance).__name__} exposes none of {joined_names}."
    )


def _component(vector: Any, name: str) -> float:
    value: Any = getattr(vector, name)
    if callable(value):
        value = value()
    return float(value)


@dataclass
class WheelAssembly:
    name: str
    body: object
    brake_motor: object
    brake_function: object
    radius: float
    max_brake_torque: float
    force_accumulator: int | None = None
    last_tire_force_N: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_tire_moment_Nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_longitudinal_slip: float = 0.0
    last_lateral_slip: float = 0.0
    last_tire_deflection_m: float = 0.0
    last_normal_force_N: float = 0.0


class ThreeWheelerModel:
    """Straight-line reverse-trike model for braking studies.

    Coordinate convention:
        +X forward, +Y left, +Z up.

    Topology:
        one center front wheel + left/right rear wheels.

    Baseline assumptions:
        - rigid wheel-center locations
        - front steering fixed at zero
        - TMeasy tire forces or fallback rigid wheel/road contact
        - braking torque applied through ChLinkMotorRotationTorque
    """

    def __init__(self):
        self.cfg = load_all()
        self.tire_model = str(
            self.cfg["tire"].get("Model", "SMC_CONTACT")
        ).strip().upper()
        self.use_tmeasy_tire = self.tire_model in {
            "TMEASY",
            "TMEASY_LOCAL",
            "TMEASY_LONGITUDINAL",
        }
        self.active_road_friction = float(
            self.cfg["tire"]["Friction_coefficient"]
        )

        self._configure_collision_defaults()
        self.system = chrono.ChSystemSMC()
        self._configure_solver()
        gravity = _chrono_vector(
            0.0,
            0.0,
            -float(self.cfg["braking"]["Gravity_mps2"]),
        )
        _call_first_method(
            self.system,
            ("SetGravitationalAcceleration", "Set_G_acc"),
            gravity,
        )

        # Collision backend compatibility across PyChrono versions.
        try:
            self.system.SetCollisionSystemType(
                chrono.ChCollisionSystem.Type_BULLET
            )
        except Exception:
            pass

        self._make_materials()
        self._make_ground()
        self._make_chassis()
        self._make_wheels()

    def _configure_collision_defaults(self):
        """Use small, explicit collision tolerances for wheel-road contact."""
        collision_model = getattr(chrono, "ChCollisionModel", None)
        if collision_model is None:
            return

        braking_cfg = self.cfg["braking"]
        envelope = float(braking_cfg.get("Collision_envelope_m", 0.001))
        margin = float(braking_cfg.get("Collision_margin_m", 0.001))

        set_envelope = getattr(
            collision_model,
            "SetDefaultSuggestedEnvelope",
            None,
        )
        if callable(set_envelope):
            set_envelope(envelope)

        set_margin = getattr(
            collision_model,
            "SetDefaultSuggestedMargin",
            None,
        )
        if callable(set_margin):
            set_margin(margin)

    def _configure_solver(self):
        """Configure the iterative solve for the wheel joint constraints."""
        solver = self.system.GetSolver()
        as_iterative = getattr(solver, "AsIterative", None)
        if callable(as_iterative):
            iterative_solver = as_iterative()
            if iterative_solver is not None:
                solver = iterative_solver

        braking_cfg = self.cfg["braking"]
        set_iterations = getattr(solver, "SetMaxIterations", None)
        if callable(set_iterations):
            set_iterations(int(braking_cfg.get("Solver_max_iterations", 200)))

        set_tolerance = getattr(solver, "SetTolerance", None)
        if callable(set_tolerance):
            set_tolerance(float(braking_cfg.get("Solver_tolerance", 1.0e-10)))

    # ------------------------------------------------------------------
    # Small compatibility helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _set_lin_vel(body, velocity):
        """Set body linear velocity using the API available in this build."""
        if hasattr(body, "SetLinVel"):
            body.SetLinVel(velocity)
        elif hasattr(body, "SetPosDt"):
            body.SetPosDt(velocity)
        else:
            raise AttributeError(
                f"{type(body).__name__} has neither SetLinVel() nor SetPosDt()."
            )

    @staticmethod
    def _get_lin_vel(body):
        """Get body linear velocity using the API available in this build."""
        if hasattr(body, "GetLinVel"):
            return body.GetLinVel()
        if hasattr(body, "GetPosDt"):
            return body.GetPosDt()

        raise AttributeError(
            f"{type(body).__name__} has neither GetLinVel() nor GetPosDt()."
        )

    @staticmethod
    def _set_ang_vel_local(body, omega_local):
        """Set angular velocity expressed in the body's local frame."""
        if hasattr(body, "SetAngVelLocal"):
            body.SetAngVelLocal(omega_local)
            return

        # Fallback for builds that expose only parent-frame angular velocity.
        if hasattr(body, "SetAngVelParent"):
            omega_parent = body.TransformDirectionLocalToParent(omega_local)
            body.SetAngVelParent(omega_parent)
            return

        raise AttributeError(
            f"{type(body).__name__} has neither SetAngVelLocal() "
            "nor SetAngVelParent()."
        )

    @staticmethod
    def _get_ang_vel_local(body):
        """Get angular velocity expressed in the body's local frame."""
        if hasattr(body, "GetAngVelLocal"):
            return body.GetAngVelLocal()

        if hasattr(body, "GetAngVelParent"):
            return body.TransformDirectionParentToLocal(
                body.GetAngVelParent()
            )

        raise AttributeError(
            f"{type(body).__name__} has neither GetAngVelLocal() "
            "nor GetAngVelParent()."
        )

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def _make_materials(self):
        tire_cfg = self.cfg["tire"]

        self.road_material = _chrono_contact_material_smc()
        self.wheel_material = _chrono_contact_material_smc()

        for mat in (self.road_material, self.wheel_material):
            mat.SetFriction(
                float(tire_cfg["Friction_coefficient"])
            )
            mat.SetRestitution(
                float(tire_cfg["Restitution"])
            )

            if hasattr(mat, "SetYoungModulus"):
                mat.SetYoungModulus(
                    float(tire_cfg.get("Young_modulus_Pa", 2.0e7))
                )

            if hasattr(mat, "SetPoissonRatio"):
                mat.SetPoissonRatio(
                    float(tire_cfg.get("Poisson_ratio", 0.3))
                )

            if hasattr(mat, "SetRollingFriction"):
                mat.SetRollingFriction(
                    float(tire_cfg["Rolling_friction"])
                )

            if hasattr(mat, "SetSpinningFriction"):
                mat.SetSpinningFriction(
                    float(tire_cfg["Spinning_friction"])
                )

    def _add_force_accumulator(self, body) -> int | None:
        add_accumulator = getattr(body, "AddAccumulator", None)
        if callable(add_accumulator):
            return int(add_accumulator())
        return None

    def _empty_force_accumulator(self, body, force_accumulator: int | None):
        if force_accumulator is not None:
            empty_accumulator = getattr(body, "EmptyAccumulator", None)
            if callable(empty_accumulator):
                empty_accumulator(force_accumulator)
                return

        for method_name in ("EmptyAccumulators", "Empty_forces_accumulators"):
            method = getattr(body, method_name, None)
            if callable(method):
                method()
                return

    def _accumulate_force(
        self,
        body,
        force_accumulator: int | None,
        force,
        point,
        local: bool = False,
    ):
        if force_accumulator is not None:
            method = getattr(body, "AccumulateForce", None)
            if callable(method):
                try:
                    method(force_accumulator, force, point, bool(local))
                    return
                except TypeError:
                    pass

        for method_name in ("AccumulateForce", "Accumulate_force"):
            method = getattr(body, method_name, None)
            if callable(method):
                method(force, point, bool(local))
                return

        raise AttributeError(
            f"{type(body).__name__} exposes no compatible force accumulator."
        )

    def _accumulate_torque(
        self,
        body,
        force_accumulator: int | None,
        torque,
        local: bool = False,
    ):
        if force_accumulator is not None:
            method = getattr(body, "AccumulateTorque", None)
            if callable(method):
                try:
                    method(force_accumulator, torque, bool(local))
                    return
                except TypeError:
                    pass

        for method_name in ("AccumulateTorque", "Accumulate_torque"):
            method = getattr(body, method_name, None)
            if callable(method):
                method(torque, bool(local))
                return

        raise AttributeError(
            f"{type(body).__name__} exposes no compatible torque accumulator."
        )

    @staticmethod
    def _set_collision_enabled(body, enabled: bool):
        for method_name in ("EnableCollision", "SetCollide"):
            method = getattr(body, method_name, None)
            if callable(method):
                method(bool(enabled))
                return

    def _make_ground(self):
        bcfg = self.cfg["braking"]

        length = float(bcfg["Road_length_m"])
        width = float(bcfg["Road_width_m"])
        thickness = float(bcfg["Road_thickness_m"])

        self.ground = chrono.ChBodyEasyBox(
            length,
            width,
            thickness,
            1000.0,
            True,
            True,
            self.road_material,
        )

        self.ground.SetName("Road")
        self.ground.SetPos(
            _chrono_vector(
                length / 2.0 - 20.0,
                0.0,
                -thickness / 2.0,
            )
        )
        self.ground.SetFixed(True)

        self.system.AddBody(self.ground)

    def _make_chassis(self):
        ccfg = self.cfg["chassis"]

        dims = ccfg["Visualization_box_m"]
        com = ccfg["COM_from_rear_axle_m"]
        lane_y = float(
            self.cfg["braking"]["Initial_lane_y_m"]
        )

        self.chassis = chrono.ChBodyEasyBox(
            float(dims[0]),
            float(dims[1]),
            float(dims[2]),
            1000.0,
            True,
            False,
        )

        self.chassis.SetName("Chassis")
        self.chassis.SetMass(
            float(ccfg["Mass_kg"])
        )

        inertia = ccfg["Inertia_kg_m2"]
        self.chassis.SetInertiaXX(
            _chrono_vector(
                float(inertia[0]),
                float(inertia[1]),
                float(inertia[2]),
            )
        )

        self.chassis.SetPos(
            _chrono_vector(
                float(com[0]),
                lane_y + float(com[1]),
                float(com[2]),
            )
        )

        yaw = math.radians(
            float(ccfg.get("Initial_yaw_deg", 0.0))
        )

        if abs(yaw) > 0.0:
            self.chassis.SetRot(
                chrono.QuatFromAngleZ(yaw)
            )

        self.system.AddBody(self.chassis)

    def _new_wheel(
        self,
        name: str,
        position,
        radius: float,
        width: float,
        mass: float,
        inertia,
        max_brake_torque: float,
    ) -> WheelAssembly:
        """Create one rigid wheel and its brake torque motor."""

        # Cylinder axis is local Y, which is the wheel spin axis.
        wheel = _chrono_easy_cylinder(
            float(radius),
            float(width),
            1000.0,
            True,
            not self.use_tmeasy_tire,
            self.wheel_material,
        )

        wheel.SetName(name)
        wheel.SetMass(float(mass))

        wheel.SetInertiaXX(
            _chrono_vector(
                float(inertia[0]),
                float(inertia[1]),
                float(inertia[2]),
            )
        )

        wheel.SetPos(
            _chrono_vector(
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
        )

        self.system.AddBody(wheel)
        if self.use_tmeasy_tire:
            self._set_collision_enabled(wheel, False)

        force_accumulator = self._add_force_accumulator(wheel)

        # ChLinkMotorRotationTorque uses local Z as its rotation axis.
        # Rotate that local Z axis so it coincides with global/local wheel Y.
        brake_motor = chrono.ChLinkMotorRotationTorque()

        motor_frame = _chrono_frame(
            _chrono_vector(
                float(position[0]),
                float(position[1]),
                float(position[2]),
            ),
            _chrono_rotate_z_to_y(),
        )

        brake_motor.Initialize(
            wheel,
            self.chassis,
            motor_frame,
        )

        brake_motor.SetName(
            name + "_Brake"
        )

        brake_function = _chrono_const_function(0.0)
        brake_motor.SetTorqueFunction(
            brake_function
        )

        self.system.AddLink(brake_motor)

        return WheelAssembly(
            name=name,
            body=wheel,
            brake_motor=brake_motor,
            brake_function=brake_function,
            radius=float(radius),
            max_brake_torque=float(max_brake_torque),
            force_accumulator=force_accumulator,
        )

    def _make_wheels(self):
        fcfg = self.cfg["front_wheel"]
        rcfg = self.cfg["rear_wheel"]
        scfg = self.cfg["rear_suspension"]

        lane_y = float(
            self.cfg["braking"]["Initial_lane_y_m"]
        )

        front_r = float(
            fcfg["Radius_m"]
        )
        rear_r = float(
            rcfg["Radius_m"]
        )

        front_x = float(
            fcfg["Wheelbase_m"]
        )
        rear_x = float(
            scfg["Rear_axle_x_m"]
        )

        half_track = (
            float(scfg["Rear_track_m"]) / 2.0
        )

        self.wheels: Dict[str, WheelAssembly] = {}

        # Front center wheel
        self.wheels["Wheel_F"] = self._new_wheel(
            name="Wheel_F",
            position=(
                front_x,
                lane_y + float(
                    fcfg["Lateral_offset_m"]
                ),
                front_r,
            ),
            radius=front_r,
            width=float(
                fcfg["Width_m"]
            ),
            mass=float(
                fcfg["Mass_kg"]
            ),
            inertia=fcfg["Inertia_kg_m2"],
            max_brake_torque=float(
                fcfg["Brake_max_torque_Nm"]
            ),
        )

        # Rear-left wheel
        self.wheels["Wheel_RL"] = self._new_wheel(
            name="Wheel_RL",
            position=(
                rear_x,
                lane_y + half_track,
                rear_r,
            ),
            radius=rear_r,
            width=float(
                rcfg["Width_m"]
            ),
            mass=float(
                rcfg["Mass_each_kg"]
            ),
            inertia=rcfg["Inertia_each_kg_m2"],
            max_brake_torque=float(
                rcfg["Brake_max_torque_each_Nm"]
            ),
        )

        # Rear-right wheel
        self.wheels["Wheel_RR"] = self._new_wheel(
            name="Wheel_RR",
            position=(
                rear_x,
                lane_y - half_track,
                rear_r,
            ),
            radius=rear_r,
            width=float(
                rcfg["Width_m"]
            ),
            mass=float(
                rcfg["Mass_each_kg"]
            ),
            inertia=rcfg["Inertia_each_kg_m2"],
            max_brake_torque=float(
                rcfg["Brake_max_torque_each_Nm"]
            ),
        )

    # ------------------------------------------------------------------
    # State and controls
    # ------------------------------------------------------------------
    def set_contact_friction(self, coefficient: float):
        """Set road friction without rebuilding the tire/contact model."""
        friction = max(float(coefficient), 0.0)
        self.active_road_friction = friction
        for material in (self.road_material, self.wheel_material):
            material.SetFriction(friction)

    def set_initial_speed(self, speed_mps: float):
        """Initialize the vehicle in approximately pure rolling.

        Chassis and all wheel centers receive the same +X translational
        velocity.  Each wheel then receives +Y angular velocity such that

            omega * rolling_radius = V

        for forward rolling.
        """

        speed = float(speed_mps)
        vehicle_velocity = _chrono_vector(
            speed,
            0.0,
            0.0,
        )

        # Chassis linear velocity.
        self._set_lin_vel(
            self.chassis,
            vehicle_velocity,
        )
        self._set_ang_vel_local(
            self.chassis,
            _chrono_vector(0.0, 0.0, 0.0),
        )

        # Wheel translational and angular velocities.
        for assembly in self.wheels.values():
            wheel = assembly.body

            self._set_lin_vel(
                wheel,
                vehicle_velocity,
            )

            rolling_radius = assembly.radius
            if self.use_tmeasy_tire:
                rolling_radius = max(
                    assembly.radius - assembly.last_tire_deflection_m / 3.0,
                    0.5 * assembly.radius,
                )

            omega = speed / rolling_radius

            self._set_ang_vel_local(
                wheel,
                _chrono_vector(
                    0.0,
                    omega,
                    0.0,
                ),
            )

    def get_forward_speed(self) -> float:
        """Return chassis longitudinal speed in chassis coordinates."""

        v_parent = self._get_lin_vel(
            self.chassis
        )

        v_local = (
            self.chassis.TransformDirectionParentToLocal(
                v_parent
            )
        )

        return _component(v_local, "x")

    def set_brake_input(self, brake_input: float):
        """Apply braking torque opposite to each wheel's spin direction."""

        u = max(
            0.0,
            min(
                1.0,
                float(brake_input),
            ),
        )

        for assembly in self.wheels.values():

            # Prefer motor-relative speed because brake torque acts through
            # the motor/revolute connection.
            try:
                omega = float(
                    _call_first_method(
                        assembly.brake_motor,
                        ("GetMotorAngleDt",),
                    )
                )
            except Exception:
                omega_local = self._get_ang_vel_local(
                    assembly.body
                )
                omega = _component(omega_local, "y")

            # Avoid rapidly reversing brake torque when the wheel is nearly
            # stopped.
            if abs(omega) < 0.25:
                torque = 0.0
            else:
                torque = -math.copysign(
                    u * assembly.max_brake_torque,
                    omega,
                )

            # ChFunctionConst API differs slightly across some builds.
            _call_first_method(
                assembly.brake_function,
                ("SetConstant", "SetValue"),
                torque,
            )

    # ------------------------------------------------------------------
    # Simulation helpers
    # ------------------------------------------------------------------
    def compute_contact_forces(self):
        container = (
            self.system.GetContactContainer()
        )

        if hasattr(
            container,
            "ComputeContactForces",
        ):
            container.ComputeContactForces()

    def advance(self, step_size: float):
        if self.use_tmeasy_tire:
            self._apply_tmeasy_tire_forces(float(step_size))

        self.system.DoStepDynamics(
            float(step_size)
        )

    def get_time(self) -> float:
        return float(
            self.system.GetChTime()
        )

    def total_mass(self) -> float:
        wheel_mass = sum(
            float(
                _call_first_method(
                    assembly.body,
                    ("GetMass",),
                )
            )
            for assembly
            in self.wheels.values()
        )

        return (
            float(
                _call_first_method(
                    self.chassis,
                    ("GetMass",),
                )
            )
            + wheel_mass
        )

    # ------------------------------------------------------------------
    # TMeasy tire forces
    # ------------------------------------------------------------------
    @staticmethod
    def _interp_l(q: float, value_at_nominal: float, value_at_double: float) -> float:
        return value_at_nominal + (value_at_double - value_at_nominal) * (q - 1.0)

    @staticmethod
    def _interp_q(q: float, value_at_nominal: float, value_at_double: float) -> float:
        return q * (
            2.0 * value_at_nominal
            - 0.5 * value_at_double
            - (value_at_nominal - 0.5 * value_at_double) * q
        )

    @staticmethod
    def _tmeasy_curve(
        slip: float,
        initial_slope: float,
        slip_at_peak: float,
        peak_force: float,
        slip_at_slide: float,
        sliding_force: float,
    ) -> float:
        if (
            slip <= 0.0
            or initial_slope <= 0.0
            or slip_at_peak <= 0.0
            or peak_force <= 0.0
            or slip_at_slide <= slip_at_peak
        ):
            return 0.0

        slope = max(2.0 * peak_force / slip_at_peak, initial_slope)
        if slip > slip_at_slide:
            return sliding_force

        if slip < slip_at_peak:
            p = slope * slip_at_peak / peak_force - 2.0
            normalized_slip = slip / slip_at_peak
            denominator = 1.0 + (normalized_slip + p) * normalized_slip
            return slope * slip_at_peak * normalized_slip / denominator

        a = (peak_force / slip_at_peak) ** 2 / (slope * slip_at_peak)
        connecting_slip = slip_at_peak + (
            (peak_force - sliding_force)
            / (a * (slip_at_slide - slip_at_peak))
        )

        if connecting_slip <= slip_at_slide:
            if slip <= connecting_slip:
                return peak_force - a * (slip - slip_at_peak) ** 2

            b = a * (connecting_slip - slip_at_peak) / (
                slip_at_slide - connecting_slip
            )
            return sliding_force + b * (slip_at_slide - slip) ** 2

        normalized_slip = (slip - slip_at_peak) / (
            slip_at_slide - slip_at_peak
        )
        return peak_force - (
            (peak_force - sliding_force)
            * normalized_slip
            * normalized_slip
            * (3.0 - 2.0 * normalized_slip)
        )

    def _horizontal_tire_axes(self):
        forward = self.chassis.TransformDirectionLocalToParent(
            _chrono_vector(1.0, 0.0, 0.0)
        )
        forward_x = _component(forward, "x")
        forward_y = _component(forward, "y")
        length = math.hypot(forward_x, forward_y)
        if length <= 1.0e-12:
            return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)

        x_axis = (
            forward_x / length,
            forward_y / length,
            0.0,
        )
        y_axis = (
            -x_axis[1],
            x_axis[0],
            0.0,
        )
        return x_axis, y_axis

    @staticmethod
    def _dot_parent_vector(vector, axis: tuple[float, float, float]) -> float:
        return (
            _component(vector, "x") * axis[0]
            + _component(vector, "y") * axis[1]
            + _component(vector, "z") * axis[2]
        )

    def _tmeasy_axis_parameters(self, axis_prefix: str, normal_force: float):
        tire_cfg = self.cfg["tire"]
        nominal_load = float(tire_cfg["Nominal_load_N"])
        maximum_load = float(
            tire_cfg.get("Maximum_load_N", 2.0 * nominal_load)
        )
        q = min(max(normal_force, 0.0), maximum_load) / nominal_load

        stiffness_per_load = float(
            tire_cfg[f"{axis_prefix}_initial_stiffness_per_unit_load"]
        )
        stiffness_scale_2pn = float(
            tire_cfg.get(
                f"Double_load_{axis_prefix}_stiffness_scale",
                1.8,
            )
        )
        peak_slip_pn = float(tire_cfg[f"Peak_{axis_prefix}_slip"])
        peak_slip_2pn = float(
            tire_cfg.get(
                f"Double_load_peak_{axis_prefix}_slip",
                peak_slip_pn,
            )
        )
        peak_mu = float(tire_cfg[f"Peak_{axis_prefix}_mu"])
        peak_mu_scale_2pn = float(
            tire_cfg.get(
                f"Double_load_peak_{axis_prefix}_mu_scale",
                0.95,
            )
        )
        slide_slip_pn = float(tire_cfg[f"Sliding_{axis_prefix}_slip"])
        slide_slip_2pn = float(
            tire_cfg.get(
                f"Double_load_sliding_{axis_prefix}_slip",
                slide_slip_pn,
            )
        )
        slide_mu = float(tire_cfg[f"Sliding_{axis_prefix}_mu"])
        slide_mu_scale_2pn = float(
            tire_cfg.get(
                f"Double_load_sliding_{axis_prefix}_mu_scale",
                0.9,
            )
        )

        d0_pn = stiffness_per_load * nominal_load
        d0_p2n = d0_pn * stiffness_scale_2pn
        peak_pn = peak_mu * nominal_load
        peak_p2n = peak_mu * peak_mu_scale_2pn * 2.0 * nominal_load
        slide_pn = slide_mu * nominal_load
        slide_p2n = slide_mu * slide_mu_scale_2pn * 2.0 * nominal_load

        return (
            self._interp_q(q, d0_pn, d0_p2n),
            self._interp_l(q, peak_slip_pn, peak_slip_2pn),
            self._interp_q(q, peak_pn, peak_p2n),
            self._interp_l(q, slide_slip_pn, slide_slip_2pn),
            self._interp_q(q, slide_pn, slide_p2n),
        )

    def _apply_tmeasy_tire_forces(self, step_size: float):
        del step_size  # The current local TMeasy law is quasi-steady.
        tire_cfg = self.cfg["tire"]
        road_z = 0.0
        nominal_load = float(tire_cfg["Nominal_load_N"])
        maximum_load = float(
            tire_cfg.get("Maximum_load_N", 2.0 * nominal_load)
        )
        vertical_stiffness = float(tire_cfg["Vertical_stiffness_N_per_m"])
        vertical_quadratic = float(
            tire_cfg.get("Vertical_quadratic_stiffness_N_per_m2", 0.0)
        )
        vertical_damping = float(tire_cfg["Vertical_damping_Ns_per_m"])
        bottoming_deflection = float(
            tire_cfg.get("Bottoming_deflection_m", 1.0e9)
        )
        bottoming_stiffness = float(
            tire_cfg.get("Bottoming_stiffness_N_per_m", 0.0)
        )
        reference_mu = max(float(tire_cfg["Reference_friction"]), 1.0e-9)
        mu_scale = self.active_road_friction / reference_mu
        transport_regularization = float(
            tire_cfg.get("Low_speed_transport_mps", 0.01)
        )
        rolling_resistance = float(
            tire_cfg.get("Rolling_resistance_coefficient", 0.0)
        )
        x_axis, y_axis = self._horizontal_tire_axes()

        for assembly in self.wheels.values():
            wheel = assembly.body
            self._empty_force_accumulator(
                wheel,
                assembly.force_accumulator,
            )

            pos = _call_first_method(wheel, ("GetPos",))
            lin_vel_parent = self._get_lin_vel(wheel)
            local_omega = self._get_ang_vel_local(wheel)

            center_z = _component(pos, "z")
            depth = max(assembly.radius - (center_z - road_z), 0.0)
            depth_rate = max(-_component(lin_vel_parent, "z"), 0.0)

            normal_force = (
                vertical_stiffness * depth
                + vertical_quadratic * depth * depth
                + vertical_damping * depth_rate
            )
            if depth > bottoming_deflection:
                normal_force += bottoming_stiffness * (
                    depth - bottoming_deflection
                )

            normal_force = max(normal_force, 0.0)

            if depth <= 0.0 or normal_force <= 0.0:
                assembly.last_tire_force_N = (0.0, 0.0, 0.0)
                assembly.last_tire_moment_Nm = (0.0, 0.0, 0.0)
                assembly.last_longitudinal_slip = 0.0
                assembly.last_lateral_slip = 0.0
                assembly.last_tire_deflection_m = depth
                assembly.last_normal_force_N = 0.0
                continue

            fz_for_curve = min(normal_force, maximum_load)
            effective_radius = max(
                assembly.radius - depth / 3.0,
                0.5 * assembly.radius,
            )
            omega_y = _component(local_omega, "y")
            vx = self._dot_parent_vector(lin_vel_parent, x_axis)
            vy = self._dot_parent_vector(lin_vel_parent, y_axis)
            transport_speed = (
                effective_radius * abs(omega_y)
                + transport_regularization
            )
            sx = -(vx - omega_y * effective_radius) / transport_speed
            sy = -vy / transport_speed
            combined_slip = math.hypot(sx, sy)

            if combined_slip > 0.0 and mu_scale > 0.0:
                cos_slip = sx / combined_slip
                sin_slip = sy / combined_slip

                x_params = self._tmeasy_axis_parameters(
                    "longitudinal",
                    fz_for_curve,
                )
                y_params = self._tmeasy_axis_parameters(
                    "lateral",
                    fz_for_curve,
                )

                dfx0, sxm, fxm, sxs, fxs = x_params
                dfy0, sym, fym, sys, fys = y_params

                initial_slope = math.hypot(
                    dfx0 * cos_slip,
                    dfy0 * sin_slip,
                )
                peak_force = mu_scale * math.hypot(
                    fxm * cos_slip,
                    fym * sin_slip,
                )
                peak_slip = mu_scale * math.hypot(
                    sxm * cos_slip,
                    sym * sin_slip,
                )
                sliding_force = mu_scale * math.hypot(
                    fxs * cos_slip,
                    fys * sin_slip,
                )
                sliding_slip = mu_scale * math.hypot(
                    sxs * cos_slip,
                    sys * sin_slip,
                )

                horizontal_force = self._tmeasy_curve(
                    combined_slip,
                    initial_slope,
                    peak_slip,
                    peak_force,
                    sliding_slip,
                    sliding_force,
                )
                fx = horizontal_force * cos_slip
                fy = horizontal_force * sin_slip
            else:
                fx = 0.0
                fy = 0.0

            force_parent = _chrono_vector(
                fx * x_axis[0] + fy * y_axis[0],
                fx * x_axis[1] + fy * y_axis[1],
                normal_force,
            )
            contact_point = _chrono_vector(
                _component(pos, "x"),
                _component(pos, "y"),
                road_z,
            )
            self._accumulate_force(
                wheel,
                assembly.force_accumulator,
                force_parent,
                contact_point,
                False,
            )

            rolling_torque = -rolling_resistance * normal_force * assembly.radius
            rolling_torque *= math.tanh(omega_y)
            local_moment = _chrono_vector(
                0.0,
                rolling_torque,
                0.0,
            )
            self._accumulate_torque(
                wheel,
                assembly.force_accumulator,
                local_moment,
                True,
            )

            assembly.last_tire_force_N = (
                _component(force_parent, "x"),
                _component(force_parent, "y"),
                _component(force_parent, "z"),
            )
            assembly.last_tire_moment_Nm = (
                0.0,
                rolling_torque,
                0.0,
            )
            assembly.last_longitudinal_slip = sx
            assembly.last_lateral_slip = sy
            assembly.last_tire_deflection_m = depth
            assembly.last_normal_force_N = normal_force

    def get_wheel_road_force(self, wheel_name: str):
        assembly = self.wheels[wheel_name]
        if self.use_tmeasy_tire:
            return assembly.last_tire_force_N

        contact = _call_first_method(assembly.body, ("GetContactForce",))
        return (
            _component(contact, "x"),
            _component(contact, "y"),
            _component(contact, "z"),
        )
