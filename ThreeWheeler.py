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


class ThreeWheelerModel:
    """Straight-line reverse-trike model for braking studies.

    Coordinate convention:
        +X forward, +Y left, +Z up.

    Topology:
        one center front wheel + left/right rear wheels.

    Baseline assumptions:
        - rigid wheel-center locations
        - front steering fixed at zero
        - rigid wheel/road contact
        - braking torque applied through ChLinkMotorRotationTorque
    """

    def __init__(self):
        self.cfg = load_all()

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
            True,
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
        """Set wheel and road friction without rebuilding the contact model."""
        friction = max(float(coefficient), 0.0)
        for material in (self.road_material, self.wheel_material):
            material.SetFriction(friction)

    def set_initial_speed(self, speed_mps: float):
        """Initialize the vehicle in approximately pure rolling.

        Chassis and all wheel centers receive the same +X translational
        velocity.  Each wheel then receives +Y angular velocity such that

            omega * R = V

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

            omega = speed / assembly.radius

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
