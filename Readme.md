# Project Chrono straight-line braking workflow

This baseline is a repeatable **100-to-0 km/h Sedan full-brake stop** on flat, high-friction pavement. It uses Chrono's packaged Sedan with TMeasy tires and automatically produces a graph from which stopping time and stopping distance can be read directly.

## Representative setup

| Item | Baseline value | Why it matters |
|---|---:|---|
| Vehicle | `Sedan` | Packaged representative passenger-car model |
| Driveline | Sedan default | Model-defined passenger-car driveline |
| Tire | TMeasy | Captures longitudinal slip/load behavior; better for braking than rigid tires |
| Road | Rigid, flat, `mu = 0.90` | Dry-asphalt-like baseline |
| Initial test speed | 100 km/h | Standard, readily interpreted test condition |
| Brake rise time | 0.25 s | Avoids a numerically abrupt step and represents pedal/application buildup |
| Brake demand | 1.0 normalized | Maximum model brake torque |
| MBS/tire steps | 2 ms / 1 ms | Practical starting point; verify by time-step convergence |

The normalized Chrono braking input is not hydraulic pressure. It scales the brake subsystem's available torque. To match a real vehicle, replace the packaged brake parameters with measured maximum axle brake torques, balance, and actuation dynamics.

## Decoupled Execution Workflow

The simulation uses a data-driven, decoupled architecture. This separates the heavy physics calculations from post-processing and 3D rendering. All necessary vehicle models, mesh assets (`.obj`, `.mtl`), subsystem configurations (`.json`), and tire specifications (`.tir`) are stored locally within the repository under `data/vehicle/sedan/`, making the project fully standalone and portable without relying on external Chrono installation data directories.

### Local Data Directory Structure
```
data/
└── vehicle/
    └── sedan/
        ├── Sedan.json
        ├── sedan_chassis_vis.obj / .mtl
        ├── sedan_rim.obj / .mtl
        ├── sedan_tire.obj / .mtl
        ├── brake/
        │   ├── Sedan_BrakeShafts.json
        │   └── Sedan_BrakeSimple.json
        ├── chassis/
        │   └── Sedan_Chassis.json
        ├── driveline/
        │   └── Sedan_Driveline2WD.json
        ├── driver/
        │   └── Sedan_AIDriver.json
        ├── powertrain/
        │   ├── Sedan_AutomaticTransmissionSimpleMap.json
        │   ├── Sedan_EngineShafts.json
        │   ├── Sedan_EngineSimpleMap.json
        │   └── Sedan_ManualTransmissionShafts.json
        ├── steering/
        │   └── Sedan_RackPinion.json
        ├── suspension/
        │   ├── Sedan_DoubleWishbone.json
        │   └── Sedan_MultiLink.json
        ├── tire/
        │   ├── Sedan_Pac02Tire.json / .tir
        │   ├── Sedan_RigidTire.json
        │   └── Sedan_TMeasyTire.json
        ├── vehicle/
        │   └── Sedan_Vehicle.json
        └── wheel/
            └── Sedan_Wheel.json
```

### 1. Run the Headless Simulation
Execute the primary physics script in your standard Python/Conda environment:

```bash
python simulation.py
```

Expected Output: The script will create a `results/ directory` containing `simulation_telemetry.csv`.

### Stage 2: Data Analysis & Graphing (On-Demand)
This script loads the telemetry artifact to generate analysis graphs on-demand without recalculating the physics.

In the same terminal, run the following command:
```bash
python graph_generation.py
```

Expected Output: The script will save `braking_performance.png` inside the `results/ folder`, clearly annotated with the final stopping time and distance.

### Stage 3: Blender 3D Animation Generation
This script creates a `.blend` file completely from scratch. It builds placeholder 3D geometry for the chassis and wheels, reads the `simulation_telemetry.csv`, and inserts precise location and rotation keyframes.

This script requires the bpy module and uses Blender's background executable to generate a series of temporary .png and then .mp4.

You can install bpy modules using the command `pip install bpy`.

```bash
python blender_export.py
```

Expected Output: The terminal will output SUCCESS! Blender file generated, and an `animated_sedan.blend` file will appear in your results/ folder.

### Stage 4: Render the animation using Blender
Use the script `render_blender.py` to generate the `.png` sequence and `.mp4` files of the animation of the simulation as a video. This workflow could be improved and video file could be directly generated but requires further tweaking with bpy. For now, in the interest of time, video is generated through image sequence.

```bash
python render_blender.py
```

### General Workflow Principles

1. **Define the maneuver:** Set target speed, road friction, brake rise time, and stop threshold. Keep steering at zero.

2. **Assemble the plant:** Select the vehicle, powertrain, driveline, brake, and tire models.

3. **Create the environment:** Use a long rigid patch first. Later vary friction, grade, split-mu, and surface compliance.

4. **Condition the vehicle:** Accelerate to 100 km/h. At the target-speed crossing, release the throttle and begin the prescribed brake ramp.

5. **Apply brakes:** Ramp normalized braking from 0 to 1.

6. **Synchronize and advance:** At every step call terrain synchronization, vehicle synchronization with driver inputs, then advance both modules.

7. **Log responses:** Record time, longitudinal position/speed/acceleration, pitch, brake demand, wheel angular speeds, and tire forces.

8. **Calculate metrics:** Measure from the start of brake application to find stopping time/distance and mean/peak deceleration.

9. **Verify numerics:** Repeat at 2, 1, and 0.5 ms. Stopping distance should converge.

10. **Validate the model:** Compare against track data using the same speed, payload, tire pressure, and road friction.

## Sources

- [Project Chrono vehicle overview](https://api.projectchrono.org/vehicle_overview.html)
- [Project Chrono driver subsystem](https://api.projectchrono.org/vehicle_driver.html)
- [Official vehicle models reference](https://api.projectchrono.org/vehicle_models.html)
- [Chrono 10 Sedan class reference](https://api.projectchrono.org/10.0.0/classchrono_1_1vehicle_1_1sedan_1_1_sedan.html)
