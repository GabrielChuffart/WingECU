function m = motor_model(p)
%MOTOR_MODEL  Torque-speed model of the GM4108H-120T at p.bench.V_bus.
%   m = motor_model(p) returns a struct:
%     Kt       [N m/A]     torque constant
%     Ke       [V s/rad]   back-EMF constant
%     I_volt   [A]         stall current allowed by the supply voltage
%     I_max    [A]         usable current at stall, min(I_volt, I_rated)
%     T_stall  [N m]       torque at zero speed
%     T_volt   [N m]       voltage-limit line at zero speed, Kt * I_volt
%     omega0   [rad/s]     no-load speed at V_bus
%     c_short  [N m s/rad] braking coefficient, phases shorted
%     T_avail  @(w)        available motoring torque at speed w [N m]
%
%   Kt from the datasheet no-load speed, ideal motor: Kt = Ke = 1/KV (SI).
%   Optimistic case of s02, to be replaced by the measured Kt (M1).
%   T_avail(w) = min(T_stall, T_volt * (1 - |w|/omega0)):
%     flat at the current limit, then the voltage line down to omega0.
%   |w| is conservative when braking (back-EMF then helps the current).

KV        = p.motor.n0_rpm / p.motor.V_test;       % [rpm/V]
m.Kt      = 60 / (2*pi*KV);                        % [N m/A]
m.Ke      = m.Kt;                                  % [V s/rad] ideal motor, SI units
R_ph      = p.motor.R_ll / 2;                      % [ohm] phase resistance, star
m.I_volt  = (p.bench.V_bus / sqrt(3)) / R_ph;      % [A]   stall current, SVPWM
m.I_max   = min(m.I_volt, p.motor.I_rated);        % [A]
m.T_stall = m.Kt * m.I_max;                        % [N m]
m.T_volt  = m.Kt * m.I_volt;                       % [N m]
m.omega0  = p.bench.V_bus / m.Ke;                  % [rad/s] back-EMF = V_bus
m.c_short = m.T_volt / m.omega0;                   % [N m s/rad] slope of the voltage line = shorted-phase damping
m.T_avail = @(w) min(m.T_stall, m.T_volt * max(1 - abs(w)/m.omega0, 0));   % [N m]
end
