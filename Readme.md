# Three-wheeler Chrono braking and cornering baseline

This project is a **1 front + 2 rear** three-wheeler model for PyChrono, with a straight-line braking test and a constant steer-angle cornering test.
It intentionally does **not** instantiate `veh.Sedan()`, because the stock wheeled-vehicle axle/suspension abstraction assumes left/right wheel pairs.

## Files

- `ThreeWheeler.py` — constructs the custom three-wheel multibody topology.
- `braking.py` — initializes the vehicle at the target speed, ramps the brakes, logs telemetry, and writes the braking summary.
- `analyze_braking.py` — generates braking plots from the CSV.
- `ThreeWheeler_Chassis.json` — mass, inertia, COM, and simple visualization dimensions.
- `ThreeWheeler_RearSuspension.json` — rear axle position and track width; baseline is rigid.
- `ThreeWheeler_FrontWheel.json` — front wheel dimensions/inertia/brake torque.
- `ThreeWheeler_RearWheel.json` — rear wheel dimensions/inertia/brake torque.
- `ThreeWheeler_Tire.json` — TMeasy tire/road force and vertical compliance properties.
- `Braking_Config.json` — target speed, timestep, brake timing, road dimensions.
- `cornering.py`, `analyze_cornering.py`, `Cornering_Config.json` — constant steer-angle cornering test (see below).

The JSON files are **project parameter files consumed by the Python code**, not stock Chrono::Vehicle subsystem JSON schemas.

## Run

From this directory:

```bash
python braking.py
python analyze_braking.py
```

Outputs are written to `results/`.

## Important calibration parameters

Before treating results as design-quality values, replace the baseline numbers with your CAD/measured values:

1. chassis mass and inertia;
2. chassis COM relative to the rear axle;
3. wheelbase and rear track;
4. front/rear wheel radius and inertia;
5. front/rear maximum brake torque;
6. tire-road friction.

## Tire model

This version uses a local TMeasy-style force element for the custom 1-front + 2-rear topology. Wheel cylinders remain for visualization, but their tire-road collision is disabled when `ThreeWheeler_Tire.json` has `"Model": "TMEASY"`. The TMeasy force element supplies vertical tire compliance, longitudinal braking force, lateral force, and rolling resistance.

Each tire force acts at the contact point below the wheel center. It is applied at the wheel center together with its moment: the part about the axle goes to the wheel (it drives or brakes it), and the rest goes to the non-spinning body the wheel is mounted on (chassis or steering knuckle). Applying the whole moment to the spinning wheel would add a spurious yaw torque of about `wheel-center height × lateral force × spin rate × step size` whenever the tires carry lateral force.

The wheel centers are still rigidly attached to the chassis. Add front-fork/rear-suspension compliance separately if suspension dynamics are part of the study.

## Cornering: constant steer-angle test

The cornering study follows Rodriguez Licea et al. (2018), *The Rollover Risk in Delta Tricycles: A New Rollover Index and Its Robust Mitigation by Rear Differential Braking* (`references/rollover_2018.pdf`).

- `cornering.py`: settles the vehicle, runs it straight at the target speed, then steers the front wheel to a constant angle (smooth 0.2 s ramp) and holds it. A PI controller drives the rear wheels (equal torque split, i.e. an open differential) to hold the speed, as the paper assumes.
- `analyze_cornering.py`: plots the telemetry and compares it with the paper's linear lateral model (Eqs. 14, 16, 26, 27), integrated with the simulated steer angle and speed.
- `Cornering_Config.json`: speed, steering angle and timing, speed controller, wheel-lift and rollover thresholds, step size, and road/solver settings.

```bash
python cornering.py
python analyze_cornering.py

# Other conditions without editing the JSON (+ = left turn):
python cornering.py --steering-deg 11 --speed-kph 30 --output results/steer11_telemetry.csv
python analyze_cornering.py results/steer11_telemetry.csv
```

Outputs (in `results/`):

- `cornering_constant_steer_telemetry.csv`: time histories of the paper's quantities (steer angle, sideslip `beta_deg`, `yaw_rate_dps`, lateral acceleration `ay_mps2`/`ay_g`, `roll_deg`, `RI`, `RI_model`), whole-vehicle CoG motion, and per-wheel loads, forces, slip angles, and poses.
- `cornering_constant_steer_summary.json`: vehicle parameters in the paper's notation (`m`, `J`, `l_f`, `l_r`, `h`, `b`, `c_f`, `c_r`, cf. its Table 1), steady-state results, quasi-static rollover threshold, rear-wheel lift time, and rollover flag.
- `cornering_constant_steer_*.png`: inputs, lateral response vs linear model, rollover index and roll angle, tire loads/forces/slip angles, and the plan-view path.

Conventions and definitions:

- Axes follow ISO 8855 (+X forward, +Y left, +Z up). A positive steer angle, yaw rate, and lateral acceleration mean a left turn; positive roll means the vehicle leans to the right (left side up).
- Rollover index: `RI = (Fz_RR - Fz_RL) / (Fz_RR + Fz_RL)`. `+1` means the left rear wheel is unloaded (imminent rollover to the right, the outside of a left turn), `-1` the right rear wheel. This is the paper's verbal definition and matches its figures; its Eq. (17) as printed has the numerator reversed.
- `RI_model` is the paper's accelerometer estimate (Eq. 24) with the same sign convention: `RI_model = 2 h l a_y / (b g l_f) - l (b_r - b_l) / (b l_f)`. `RI_model = ±1` gives the quasi-static rollover threshold reported in the summary.
- The paper counts a rear wheel at zero vertical load as an imminent rollover. The summary records the first such lift (`wheel_lift_time_s`); the run stops once the chassis roll exceeds `Rollover_roll_angle_deg`.
- The steady state is the mean over the last `Steady_state_window_s` of a run that did not roll over. The understeer gradient is the bicycle-model equivalent `(alpha_front - mean(alpha_rear)) / a_y`.

Cornering model assumptions:

- `cornering.py` builds the model with `enable_steering=True`. The front wheel then spins on a steering knuckle that an angle motor turns about the vertical axis through the wheel center: no caster, trail, or scrub radius. The knuckle mass (`Steering_knuckle_mass_kg` in `ThreeWheeler_FrontWheel.json`) is taken from the front wheel mass, so total mass and CoG are unchanged. `braking.py` keeps the fixed front wheel. `Steering_angle_deg` in `ThreeWheeler_FrontWheel.json` is still unused; the cornering steer angle comes from `Cornering_Config.json`.
- The chassis is rigid on compliant tires (rigid rear suspension), so the only roll compliance comes from the tires. Their damping acts only in compression, so a quick steering input excites a lightly damped roll oscillation, and the transient RI can overshoot its steady value.
- With an open differential, a rear wheel that lifts off still receives half the drive torque and spins up freely.
