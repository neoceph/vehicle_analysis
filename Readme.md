# Three-wheeler Chrono braking baseline

This project is a straight-line **1 front + 2 rear** three-wheeler braking model for PyChrono.
It intentionally does **not** instantiate `veh.Sedan()`, because the stock wheeled-vehicle axle/suspension abstraction assumes left/right wheel pairs.

## Files

- `ThreeWheeler.py` — constructs the custom three-wheel multibody topology.
- `simulation.py` — initializes the vehicle at the target speed, ramps the brakes, logs telemetry, and writes the braking summary.
- `analyze_braking.py` — generates braking plots from the CSV.
- `ThreeWheeler_Chassis.json` — mass, inertia, COM, and simple visualization dimensions.
- `ThreeWheeler_RearSuspension.json` — rear axle position and track width; baseline is rigid.
- `ThreeWheeler_FrontWheel.json` — front wheel dimensions/inertia/brake torque.
- `ThreeWheeler_RearWheel.json` — rear wheel dimensions/inertia/brake torque.
- `ThreeWheeler_Tire.json` — TMeasy tire/road force and vertical compliance properties.
- `Braking_Config.json` — target speed, timestep, brake timing, road dimensions.

The JSON files are **project parameter files consumed by the Python code**, not stock Chrono::Vehicle subsystem JSON schemas.

## Run

From this directory:

```bash
python simulation.py
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

The wheel centers are still rigidly attached to the chassis. Add front-fork/rear-suspension compliance separately if suspension dynamics are part of the study.
