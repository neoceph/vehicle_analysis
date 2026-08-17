"""100-to-0 km/h Sedan braking analysis with Project Chrono / PyChrono.

Representative test:
  * Chrono Sedan, TMeasy tires
  * flat rigid road, mu = 0.9
  * accelerate to 100 km/h, 0.25 s brake ramp, full stop
  * CSV output, summary metrics, and speed/distance graphs

Run:
    python chrono_braking_analysis.py
"""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pychrono as chrono
import pychrono.vehicle as veh


# ------------------------------ test parameters ------------------------------
TARGET_SPEED = 100.0 / 3.6       # m/s
ROAD_FRICTION = 0.90
BRAKE_RAMP_TIME = 0.25           # s to ramp normalized brake input 0 -> 1
STEP_SIZE = 2.0e-3               # s, multibody integration
TIRE_STEP_SIZE = 1.0e-3          # s
MAX_SIM_TIME = 60.0              # safety cutoff
STOP_SPEED = 0.10                # m/s
OUTPUT_HZ = 200.0
results_dir = Path("results")
results_dir.mkdir(exist_ok=True)
OUTPUT_FILE = Path(results_dir / "braking_results.csv")
PLOT_FILE = Path(results_dir / "sedan_braking_graph.png")


def make_driver_inputs():
    """Handle the two names used by recent PyChrono wrapper builds."""
    cls = getattr(veh, "DriverInputs", None) or getattr(veh, "ChDriverInputs")
    return cls()


def set_inputs(inputs, steering, throttle, braking):
    inputs.m_steering = float(steering)
    inputs.m_throttle = float(throttle)
    inputs.m_braking = float(braking)


def longitudinal_speed(chassis_body):
    """Forward speed in the chassis frame; robust if the vehicle pitches."""
    v_abs = chassis_body.GetPosDt()
    v_loc = chassis_body.TransformDirectionParentToLocal(v_abs)
    return v_loc.x


def pitch_angle(chassis_body):
    """ISO Z-up pitch angle in radians from the chassis rotation matrix."""
    a = chassis_body.GetRot().GetCardanAnglesXYZ()
    return a.y


def create_model():
    sedan = veh.Sedan()
    sedan.SetContactMethod(chrono.ChContactMethod_NSC)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(0.0, 0.0, 0.5),
            chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0),
        )
    )
    sedan.SetTireType(veh.TireModelType_TMEASY)
    sedan.SetTireStepSize(TIRE_STEP_SIZE)
    sedan.Initialize()

    # No visualization assets are needed for a batch/DOE run.
    sedan.SetChassisVisualizationType(veh.VisualizationType_NONE)
    sedan.SetSuspensionVisualizationType(veh.VisualizationType_NONE)
    sedan.SetSteeringVisualizationType(veh.VisualizationType_NONE)
    sedan.SetWheelVisualizationType(veh.VisualizationType_NONE)
    sedan.SetTireVisualizationType(veh.VisualizationType_NONE)

    terrain = veh.RigidTerrain(sedan.GetSystem())
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(ROAD_FRICTION)
    patch = terrain.AddPatch(
        mat,
        chrono.ChCoordsysd(
            chrono.VNULL, chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
        ),
        1000.0,
        20.0,
    )
    patch.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    terrain.Initialize()
    return sedan, terrain


def analyze(rows, brake_start_time, brake_start_x, brake_start_speed):
    post = [r for r in rows if r[0] >= brake_start_time]
    stop = post[-1]
    decels = [-r[5] for r in post if r[5] < 0.0]
    mean_decel = (brake_start_speed**2 - stop[2]**2) / max(
        2.0 * (stop[1] - brake_start_x), 1e-9
    )
    return {
        "initial_speed_kph": 3.6 * brake_start_speed,
        "stopping_time_s": stop[0] - brake_start_time,
        "stopping_distance_m": stop[1] - brake_start_x,
        "mean_decel_mps2": mean_decel,
        "peak_decel_mps2": max(decels, default=0.0),
        "peak_decel_g": max(decels, default=0.0) / 9.80665,
        "max_abs_pitch_deg": max(abs(math.degrees(r[6])) for r in post),
    }


def make_plot(rows, brake_start_time, brake_start_x, metrics):
    """Plot only the braking portion, with brake onset as t=0 and x=0."""
    post = [r for r in rows if r[0] >= brake_start_time]
    t = [r[0] - brake_start_time for r in post]
    distance = [r[1] - brake_start_x for r in post]
    speed = [max(r[3], 0.0) for r in post]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), constrained_layout=True)
    fig.suptitle("Project Chrono Sedan: 100–0 km/h Braking")

    ax1.plot(t, speed, color="tab:blue", linewidth=2)
    ax1.scatter([metrics["stopping_time_s"]], [0], color="red", zorder=3)
    ax1.axvline(metrics["stopping_time_s"], color="red", linestyle="--", alpha=0.7)
    ax1.set(xlabel="Time after brake application (s)", ylabel="Speed (km/h)")
    ax1.set_ylim(bottom=0)
    ax1.grid(True, alpha=0.3)
    ax1.annotate(
        f'Stopping time = {metrics["stopping_time_s"]:.2f} s',
        xy=(metrics["stopping_time_s"], 0), xytext=(-150, 35),
        textcoords="offset points", arrowprops={"arrowstyle": "->"},
    )

    ax2.plot(distance, speed, color="tab:green", linewidth=2)
    ax2.scatter([metrics["stopping_distance_m"]], [0], color="red", zorder=3)
    ax2.axvline(metrics["stopping_distance_m"], color="red", linestyle="--", alpha=0.7)
    ax2.set(xlabel="Distance after brake application (m)", ylabel="Speed (km/h)")
    ax2.set_ylim(bottom=0)
    ax2.grid(True, alpha=0.3)
    ax2.annotate(
        f'Stopping distance = {metrics["stopping_distance_m"]:.2f} m',
        xy=(metrics["stopping_distance_m"], 0), xytext=(-190, 35),
        textcoords="offset points", arrowprops={"arrowstyle": "->"},
    )

    fig.savefig(PLOT_FILE, dpi=180)
    plt.close(fig)


def main():
    sedan, terrain = create_model()
    chassis = sedan.GetVehicle().GetChassisBody()
    inputs = make_driver_inputs()

    phase = "accelerate"
    phase_start = 0.0
    brake_start_time = None
    brake_start_x = None
    brake_start_speed = None
    rows = []
    last_sample_time = -1e9
    previous_speed = longitudinal_speed(chassis)
    filtered_accel = 0.0
    accel_filter_tau = 0.05

    while sedan.GetSystem().GetChTime() < MAX_SIM_TIME:
        time = sedan.GetSystem().GetChTime()
        speed = longitudinal_speed(chassis)

        # Repeatable open-loop event sequence with a speed trigger.
        if phase == "accelerate":
            set_inputs(inputs, 0.0, 1.0, 0.0)
            if speed >= TARGET_SPEED:
                # Brake onset defines t=0 and x=0 in the final graph.
                phase = "brake"
                phase_start = time
                brake_start_time = time
                brake_start_x = chassis.GetPos().x
                brake_start_speed = speed
                set_inputs(inputs, 0.0, 0.0, 0.0)
        elif phase == "brake":
            brake = min((time - phase_start) / BRAKE_RAMP_TIME, 1.0)
            set_inputs(inputs, 0.0, 0.0, brake)
            if speed <= STOP_SPEED and time - phase_start > BRAKE_RAMP_TIME:
                phase = "stopped"
        else:
            set_inputs(inputs, 0.0, 0.0, 1.0)

        # Filter differentiated speed only for reporting peak acceleration.
        raw_accel = (speed - previous_speed) / STEP_SIZE
        alpha = STEP_SIZE / (accel_filter_tau + STEP_SIZE)
        filtered_accel += alpha * (raw_accel - filtered_accel)
        previous_speed = speed

        if time - last_sample_time >= 1.0 / OUTPUT_HZ - 0.5 * STEP_SIZE:
            rows.append([
                time,
                chassis.GetPos().x,
                speed,
                3.6 * speed,
                inputs.m_braking,
                filtered_accel,
                pitch_angle(chassis),
            ])
            last_sample_time = time

        # Standard Chrono::Vehicle force-displacement co-simulation sequence.
        terrain.Synchronize(time)
        sedan.Synchronize(time, inputs, terrain)
        terrain.Advance(STEP_SIZE)
        sedan.Advance(STEP_SIZE)

        if phase == "stopped":
            break

    if brake_start_time is None:
        raise RuntimeError("Target speed was not reached before MAX_SIM_TIME.")

    with OUTPUT_FILE.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s", "x_m", "speed_mps", "speed_kph", "brake_input",
            "longitudinal_accel_mps2", "pitch_rad",
        ])
        writer.writerows(rows)

    metrics = analyze(rows, brake_start_time, brake_start_x, brake_start_speed)
    make_plot(rows, brake_start_time, brake_start_x, metrics)
    print("\nChrono Sedan 100-to-0 km/h braking summary")
    print("-" * 36)
    for key, value in metrics.items():
        print(f"{key:24s}: {value:10.4f}")
    print(f"\nCSV: {OUTPUT_FILE.resolve()}")
    print(f"Graph: {PLOT_FILE.resolve()}")


if __name__ == "__main__":
    main()