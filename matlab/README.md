# MATLAB sizing and models

| Folder | Contents |
|---|---|
| `models/` | Parameters, physical models (`hinge_moment.m`, `motor_model.m`, `bench_model.m`) and the stroke simulation (`flap_dynamics.m`, `run_stroke.m`) |
| `sizing/` | Step-by-step sizing scripts, run in order |

All parameters live in `models/wingecu_params.m`, each tagged with its origin:
(R) F1 regulations, (D) datasheet, (C) design choice, (A) assumption.

## Sizing steps

| Script | Question |
|---|---|
| `s01_hinge_moment.m` | Aerodynamic hinge moment of the reference flap, F1 scale |
| `s02_scale_factor.m` | Torque scale factor bench / F1, actuator torque check |
| `s03_inertia_spring.m` | Rotating inertia, return spring, torque budget and back-EMF speed limit (hand calculation) |
| `s04_stroke_dynamics.m` | Time-domain simulation of opening, active closing and fail-safe closing |

Run the scripts from `matlab/sizing/` (they add `../models` to the path).
