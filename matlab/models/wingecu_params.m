function p = wingecu_params()
%WINGECU_PARAMS  All parameters of the WingECU model, in one place.
%   p = wingecu_params() returns a struct with every parameter used by the
%   sizing scripts. Each value is tagged with its origin:
%     (R) F1 Technical Regulations (FIA, 2023 issue, Art. 3.10)
%     (D) manufacturer datasheet
%     (C) bench design choice
%     (A) assumption, to be refined later
%   SI units everywhere (m, s, kg, N, rad) unless the name says otherwise.

%% Air
p.rho = 1.225;              % [kg/m^3] air density, ISA sea level

%% F1 reference flap (inspiration only, not replicated)
p.f1.b          = 0.95;     % [m]   flap span. (R) DRS parts inside |Y| <= 490 mm
p.f1.c          = 0.20;     % [m]   flap chord. (A) not given as a number in the rules
p.f1.v_max      = 340/3.6;  % [m/s] top speed. (A)
p.f1.gap_closed = 0.0125;   % [m]   slot gap, flap closed. (R) between 10 and 15 mm
p.f1.gap_max    = 0.085;    % [m]   max slot gap, DRS open. (R)
p.f1.x_pivot    = 1 - 0.020/p.f1.c; % [-]  real F1 pivot / c: <= 20 mm ahead of flap TE (R), comparison only

%% Aerodynamic model of the flap (A) - teams do not publish these values
p.aero.CN_closed = 1.2;     % [-] normal-force coefficient, flap closed
p.aero.CN_open   = 0.2;     % [-] normal-force coefficient, flap fully open
p.aero.x_cp      = 0.30;    % [-] centre of pressure, fraction of chord from the leading edge

%% Bench design (C) - geometry validated 2026-10-19
p.bench.x_hinge   = 0.55;          % [-]   hinge axis, fraction of chord from the leading edge
p.bench.delta_max = deg2rad(40);   % [rad] full opening angle
p.bench.k_scale   = 1/300;         % [-]   bench torque / F1 hinge moment, provisional (s02 window)
p.bench.V_bus     = 12;            % [V]   bench supply (proposal)
p.bench.T_par     = 0.01;          % [N m] parasitic torque, cogging + bearings, both motors (A) -> measure in M1
p.bench.eta_aero  = 0.5;           % [-]   share of actuator torque for the emulated aero load
p.bench.snr_min   = 3;             % [-]   min ratio emulated load / parasitic torque

p.bench.t_close   = 0.100;         % [s]   closing time target, 10-90 % (proposal)
p.bench.t_fs      = 0.200;         % [s]   fault reaction target, flap back to closed (proposal)

%% Bench flap and rotating parts (A) -> to be set in CAD
p.bench.flap_c    = 0.10;          % [m]   printed flap chord
p.bench.flap_m    = 0.040;         % [kg]  printed flap mass (PLA shell, 150 mm span)
p.mech.m_rotor    = 0.045;         % [kg]  rotor bell + magnets, one motor
p.mech.r_rotor    = 0.021;         % [m]   mean magnet radius (motor OD 47 mm)
p.mech.J_margin   = 1.2;           % [-]   shaft, coupling, screws

%% Return spring (C) - sizing criteria, values computed in s03
p.spring.n_hold   = 2;             % [-]   preload >= n_hold * T_par (holds closed, no aero, no actuator)
p.spring.ratio    = 1.5;           % [-]   torque at delta_max / preload (low stiffness)

%% Motor iFlight GM4108H-120T (D)
p.motor.R_ll    = 11.1;                 % [ohm] line-to-line resistance
p.motor.n0_rpm  = 540;                  % [rpm] no-load speed at V_test, mid of 513-567
p.motor.V_test  = 20;                   % [V]   datasheet test voltage
p.motor.I_rated = 1.5;                  % [A]   rated current (20 V test)
p.motor.T_load  = [1200 1800]*9.81e-5;  % [N m] "load torque" 1200-1800 g.cm
p.motor.P_max   = 25;                   % [W]   max power
p.motor.mass    = 0.137;                % [kg]
end
