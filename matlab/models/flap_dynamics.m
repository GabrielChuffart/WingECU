function dx = flap_dynamics(~, x, v, mode, p, m, b)
%FLAP_DYNAMICS  Bench equation of motion, state x = [delta; omega].
%   dx = flap_dynamics(t, x, v, mode, p, m, b)
%   v     [m/s]  emulated speed
%   mode  'open'   actuator at max torque, opening direction
%         'close'  actuator at max torque, closing direction
%         'off'    driver off, phases open: no actuator torque
%         'short'  phases shorted: electrical braking -c_short*omega
%   m = motor_model(p), b = bench_model(p)
%
%   J*domega/dt = T_act - T_aero - T_spring - T_fric
%   Positive direction = opening. Aero load and spring act towards closed.
%   Beyond omega0 the actuator adds no torque (diode braking neglected).

d = x(1);                                          % [rad]   flap angle
w = x(2);                                          % [rad/s] flap speed

switch mode
    case 'open'
        T_act =  m.T_avail(w);                        % [N m] current or back-EMF limited
    case 'close'
        T_act = -m.T_avail(w);                        % [N m] current or back-EMF limited
    case 'off'
        T_act = 0;                                     % [N m] high impedance
    case 'short'
        T_act = -m.c_short * w;                        % [N m] back-EMF into shorted windings
    otherwise
        error('flap_dynamics: unknown mode "%s"', mode);
end

T_aero = p.bench.k_scale * hinge_moment(v, d, p);  % [N m] emulated load, towards closed
T_spr  = b.T_pre + b.k_s * d;                      % [N m] return spring, towards closed
T_fric = p.bench.T_par * tanh(w / p.sim.w_eps);    % [N m] Coulomb friction, smoothed at w = 0

dx = [w; (T_act - T_aero - T_spr - T_fric) / b.J];
end
