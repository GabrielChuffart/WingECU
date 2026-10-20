# MATLAB — sizing and models

| Folder | Contents |
|---|---|
| `models/` | Parameters and reusable functions (`wingecu_params.m`, `hinge_moment.m`) |
| `sizing/` | Step-by-step sizing scripts, run in order |

All parameters live in `models/wingecu_params.m`, each tagged with its origin:
(R) F1 regulations, (D) datasheet, (C) design choice, (A) assumption.

## Sizing steps

| Script | Question |
|---|---|
| `s01_hinge_moment.m` | Aerodynamic hinge moment of the reference flap, F1 scale |
| `s02_scale_factor.m` | Torque scale factor bench / F1, actuator torque check |

Run the scripts from `matlab/sizing/` (they add `../models` to the path).
