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

## Run

Install a PyChrono 10 build with the Vehicle module and ensure its data directory is configured. Then place the Python file in a writable run directory and execute:

```bash
python chrono_braking_analysis.py
```

The script writes `braking_results.csv` and `sedan_braking_graph.png`. It prints initial speed, stopping time, stopping distance, mean and filtered peak deceleration, and peak chassis pitch.

The top graph is **speed versus time after brake application**. Its red intercept gives the stopping time. The bottom graph is **speed versus distance after brake application**. Its red intercept gives the stopping distance. Both axes are shifted so brake onset is exactly zero.

## Workflow

1. **Define the maneuver.** Set target speed, road friction, brake rise time, and stop threshold. Keep steering at zero.
2. **Assemble the plant.** Select the vehicle, powertrain, driveline, brake, and tire models. For a passenger car, substitute `Sedan` or your JSON-based model.
3. **Create the environment.** Use a long rigid patch first. Later vary friction, grade, split-mu, and surface compliance.
4. **Condition the vehicle.** Accelerate to 100 km/h. At the target-speed crossing, release the throttle and begin the prescribed brake ramp.
5. **Apply brakes.** Ramp normalized braking from 0 to 1. For ABS research, replace this with wheel-slip feedback and axle/wheel torque modulation.
6. **Synchronize and advance.** At every step call terrain synchronization, vehicle synchronization with driver inputs, then advance both modules.
7. **Log responses.** At minimum record time, longitudinal position/speed/acceleration, pitch, and brake demand. Add wheel angular speeds, tire forces, suspension travel, and brake torque for detailed diagnosis.
8. **Calculate metrics.** Measure from the start of brake application: stopping time/distance, mean and peak deceleration, pitch response, and (when wheel data are added) slip ratio and lock duration.
9. **Verify numerics.** Repeat at 2, 1, and 0.5 ms. Stopping distance and peak quantities should converge; peak differentiated acceleration is especially step/filter sensitive.
10. **Validate the model.** Compare against track data using the same speed, payload, tire pressure, road friction, brake temperature, and pedal/pressure history.

## Recommended parameter study

Run a factorial sweep over road friction (`0.3, 0.6, 0.9`), speed (`50, 80, 100 km/h`), payload, brake rise time, and front/rear brake capacity. For each case retain:

- stopping distance and time;
- mean and peak deceleration;
- pitch and axle load transfer;
- wheel slip ratio and lock time;
- tire longitudinal forces and utilization;
- brake work/energy by axle.

## Important interpretation notes

- Friction coefficient set on the rigid patch is an input to the contact/terrain model; effective tire-road behavior also depends on the selected tire model.
- The packaged Sedan is a representative test vehicle, not a calibrated model of a specific production car.
- A non-ABS full-brake command may lock wheels depending on model torque and friction. Add wheel-state logging before making claims about lockup or ABS performance.
- The script estimates acceleration by low-pass filtering differentiated forward speed. Use the chassis body's modeled acceleration or sensor output if you need high-frequency jerk/peak analysis.
- For braking on a grade, extend the terrain patch orientation and report both path distance and horizontal distance.

## Sources

- [Project Chrono vehicle overview](https://api.projectchrono.org/vehicle_overview.html)
- [Project Chrono driver subsystem](https://api.projectchrono.org/vehicle_driver.html)
- [Official vehicle models reference](https://api.projectchrono.org/vehicle_models.html)
- [Chrono 10 Sedan class reference](https://api.projectchrono.org/10.0.0/classchrono_1_1vehicle_1_1sedan_1_1_sedan.html)
