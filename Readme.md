# Three-wheeler Chrono braking, cornering, and lane changes

This project is a **1 front + 2 rear** three-wheeler model for PyChrono, with straight-line braking, constant steer-angle cornering, and single/double lane-change tests.
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

Lane-change files:

- `single_lane_change.py`, `SingleLaneChange_Config.json`: one prescribed front-wheel steer/countersteer pair.
- `double_lane_change.py`, `DoubleLaneChange_Config.json`: an outbound pair, straight dwell, and reversed return pair.
- `analyze_single_lane_change.py`, `analyze_double_lane_change.py`: telemetry plots and comparisons with the paper's linear lateral model.
- `analyze_double_lange_change.py`: alternate entry point matching the spelling in the original request.
- `lane_change.py`, `analyze_lane_change.py`: shared steering, telemetry extensions, and plots; reuse the cornering simulation and analysis helpers.
- `test_lane_change.py`: steering-profile, configuration, and CSV validation tests.

The JSON files are **project parameter files consumed by the Python code**, not stock Chrono::Vehicle subsystem JSON schemas.

## Run

In order to run analysis run the corresponding routine from this directory. For example for braking analysis:

```bash
python braking.py
python analyze_braking.py
```

Outputs are written to `results/`.
`analyze_braking.py` also writes `braking_performance.png`, the two-panel speed plot with the near-zero-speed point, stopping time, and stopping distance marked.

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

## Single and double lane changes

Run simulations in your environment with PyChrono installed (for this workspace, `conda activate chrono`). The analysis scripts need NumPy, pandas, and Matplotlib, but do not import PyChrono.

```bash
python single_lane_change.py
python analyze_single_lane_change.py

python double_lane_change.py
python analyze_double_lane_change.py
```

Both simulations settle the vehicle at rest, initialize its forward speed, and hold that speed with the existing rear-wheel PI torque controller. Time zero in the CSV is after stationary settling. The maneuver is applied by the existing front-wheel steering angle motor, with no lateral-position feedback.

`Steering_angle_deg` is the signed **peak front-wheel angle**, not a steering-wheel angle. A positive value starts left; a negative value mirrors the maneuver. For one lane change of duration `T`, starting at `t0`, the angle is:

```text
q = (t - t0) / T
delta = +A * sin(2*pi*q)^2   for 0 <= q < 0.5
delta = -A * sin(2*pi*q)^2   for 0.5 <= q <= 1
delta = 0                  outside the maneuver
```

Angle and angle rate are continuous at the entry, reversal, and exit. The double lane change applies this pair, holds zero steering for `Between_lane_changes_time_s`, then applies the negative of the same pair. The default signs are therefore `+ / -` for single and `+ / - / - / +` for double, with a dwell between the two pairs.

Defaults in the separate JSON files are 30 km/h, a 5 deg peak front-wheel angle, a 1 s straight run-in, and 3 s per steer/countersteer pair. The double change has a 0.5 s dwell. The single run lasts 7 s; the double run lasts 10.5 s, including straight run-out after the input ends. Steering amplitudes above the front-wheel limit and runs too short to record the full input are rejected.

Override conditions without editing JSON:

```bash
python single_lane_change.py --steering-deg -5 --speed-kph 30 --duration-s 3 --output results/single_right_telemetry.csv
python analyze_single_lane_change.py results/single_right_telemetry.csv

python double_lane_change.py --steering-deg 6 --duration-s 2.5 --dwell-s 0.5 --max-time-s 10 --output results/double6_telemetry.csv
python analyze_double_lane_change.py results/double6_telemetry.csv
```

`--start-s` overrides the steering start time. All simulation times are in seconds. CSV paths can be absolute or relative to this project, and the plotting scripts accept an optional CSV path.

Outputs in `results/`, prefixed with `single_lane_change` or `double_lane_change`:

- `*_telemetry.csv`: the cornering telemetry columns, including commanded/actual steering, speed, sideslip, yaw rate, roll, lateral acceleration, measured `RI`, acceleration-based `RI_model`, CoG and wheel poses, tire loads, forces, and slips. Adds `maneuver_phase`, `longitudinal_displacement_m`, `lateral_displacement_m`, and `heading_change_deg` relative to the first sample.
- `*_summary.json`: vehicle parameters, steering profile and timing, input completion status, final/peak lateral displacement, final heading change, peak responses, wheel lift, and rollover status. Constant-turn steady-state results are not calculated for these transient maneuvers.
- `*_inputs.png`: commanded vs actual steering, speed hold, and drive torque.
- `*_lateral_response.png`: sideslip, yaw rate, and lateral acceleration vs the paper's linear model.
- `*_rollover.png`: measured RI, acceleration estimate, linear-model RI, normalized rear-wheel loads, and roll angle.
- `*_tires.png`: vertical tire loads, lateral forces, slip angles, and a final-value table.
- `*_trajectory.png`: an equal-scale plan view and lateral displacement over time.

The plotting scripts can also read a CSV without its summary; then they omit the linear-model overlay and show tire loads in newtons rather than normalized by weight.

### Relationship to the reference paper

Section 4 and Figures 4-8 of `references/rollover_2018.pdf` motivate the prescribed-angle, constant-speed test and its measured responses. These scripts reuse the existing analysis of Eqs. (14), (16), (24), (26), and (27), driven by the actual simulated steering angle and speed. The RI sign convention is the same as the cornering test above. No differential-braking rollover mitigation controller is applied.

The paper describes a 1:19 steering reduction and validates at 14 m/s (50.4 km/h). Our input is already the front-wheel angle, so no ratio conversion is applied. The smooth lobes here are configurable test inputs, not an exact digitization of Figure 4; the project vehicle, tires, and default speed also differ from Table 1. Lane width and return accuracy are outputs of the vehicle dynamics, not enforced geometric constraints or an ISO cone-course compliance test.

Run the focused checks in the Chrono environment:

```bash
python -m unittest -v test_lane_change
```
