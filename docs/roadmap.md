# Roadmap

Milestones following the [project proposal](proposal/ProjectProposal_GabrielChuffart_v2.pdf)

## M1: Hardware and bring-up (~35 h)
- [x] Initial bench sizing (2 × GM4108H-120T, hinge at 0.55·c, 40° opening, preloaded spring)
- [x] Hinge-moment model M_h(v, δ) in MATLAB (`s01_hinge_moment.m`)
- [x] Torque scale factor window (`s02_scale_factor.m`): k ≈ 1/300 provisional, 1/50 rejected
- [ ] Measure motor Kt/Ke and parasitic torque (fixes the scale factor)
- [ ] Inertia, return spring and opening/closing dynamics (`s03`)
- [ ] Flap, hinge, shaft supports and coupling in CAD, first print
- [ ] Board schematic and PCB in KiCad (MCU, motor driver, encoder, IMU, GNSS, CAN, INA226 power monitor)
- [ ] Firmware start on lab development boards while PCBs are made
- [ ] FreeRTOS project, field-oriented current control, encoder reading
- [ ] Flap position loop running without load
- [ ] IMU, GNSS and power monitors acquired
- [ ] Timing instrumentation (GPIO time-stamps, DWT cycle counter, SystemView)

## M2: Load emulation and controllers (~45 h)
- [ ] CAN protocol between the two boards (`firmware/common`)
- [ ] Load emulator: hinge-moment model and torque loop
- [ ] Synthetic lap profiles
- [ ] C1: PID
- [ ] C2: PID + hinge-moment feed-forward
- [ ] C3: PID + disturbance observer
- [ ] Comparison of C1 to C3: tracking, closing time, timing, energy

## M3: Real data, safety and measurement campaign (~50 h)
- [ ] Real driving data replayed on the bench
- [ ] Braking detection and end-to-end delay budget (stage by stage)
- [ ] CAN latency, task latency/jitter/execution time under load
- [ ] Fault injection: power cut, encoder lost, frozen task, lost CAN messages (target < 200 ms)
- [ ] Power and energy campaign: per cycle, holding, per lap, spring stiffness
- [ ] Final report and demo video

## Stretch goals
- [ ] S1: brake-light detection via optocoupler, delay compared with IMU detection
- [ ] S2: −30 % holding energy (spring tuning + low-current holding mode)
- [ ] S3: speed-scheduled gains with automatic tuning on the bench
