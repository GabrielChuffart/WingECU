function m = motor_model(p)
%MOTOR_MODEL  Linear torque-speed model of the GM4108H-120T at p.bench.V_bus.
%   m = motor_model(p) returns a struct:
%     Kt      [N m/A]   torque constant
%     Ke      [V s/rad] back-EMF constant
%     I_max   [A]       usable current at stall
%     T_stall [N m]     torque at zero speed
%     omega0  [rad/s]   no-load speed at V_bus
%
%   Kt from the datasheet no-load speed, ideal motor: Kt = Ke = 1/KV (SI).
%   Optimistic case of s02, to be replaced by the measured Kt (M1).
%   Available torque, motoring: T_avail(w) = T_stall * (1 - |w|/omega0).
%   Valid while I_max is voltage-limited (true at 12 V).

KV        = p.motor.n0_rpm / p.motor.V_test;       % [rpm/V]
m.Kt      = 60 / (2*pi*KV);                        % [N m/A]
m.Ke      = m.Kt;                                  % [V s/rad] ideal motor, SI units
R_ph      = p.motor.R_ll / 2;                      % [ohm] phase resistance, star
I_volt    = (p.bench.V_bus / sqrt(3)) / R_ph;      % [A]   stall current, SVPWM
m.I_max   = min(I_volt, p.motor.I_rated);          % [A]
m.T_stall = m.Kt * m.I_max;                        % [N m]
m.omega0  = p.bench.V_bus / m.Ke;                  % [rad/s] back-EMF = V_bus
end
