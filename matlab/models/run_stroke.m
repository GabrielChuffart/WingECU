function r = run_stroke(v, mode, p, m, b)
%RUN_STROKE  One full stroke from rest, until the end stop or p.sim.t_end.
%   r = run_stroke(v, mode, p, m, b)
%   v     [m/s]  emulated speed, constant during the stroke
%   mode  'open' starts closed; 'close', 'off', 'short' start fully open
%
%   r.t, r.delta, r.omega   time histories [s], [rad], [rad/s]
%   r.t_10_90  [s]      time from 10 % to 90 % of the stroke (proposal definition)
%   r.t_stop   [s]      time to reach the end stop (NaN if not reached)
%   r.w_stop   [rad/s]  speed at the end stop (impact)

dmax = p.bench.delta_max;
if strcmp(mode, 'open')
    x0 = [0; 0];      target = dmax;               % closed -> open
else
    x0 = [dmax; 0];   target = 0;                  % open -> closed
end

opts = odeset('Events', @(t, x) end_stop(t, x, target), ...
              'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 1e-3);
[t, x] = ode45(@(t, x) flap_dynamics(t, x, v, mode, p, m, b), [0 p.sim.t_end], x0, opts);

r.t     = t;
r.delta = x(:, 1);
r.omega = x(:, 2);

s = abs(r.delta - x0(1)) / dmax;                   % [-] stroke fraction, 0 -> 1
r.t_10_90 = cross_time(t, s, 0.9) - cross_time(t, s, 0.1);

reached  = abs(r.delta(end) - target) < 1e-4;      % end stop hit before t_end
r.t_stop = NaN;  r.w_stop = NaN;
if reached
    r.t_stop = t(end);
    r.w_stop = r.omega(end);
end
end

function [val, isterminal, direction] = end_stop(~, x, target)
val        = x(1) - target;                        % [rad] zero at the end stop
isterminal = 1;                                    % stop integration
direction  = 0;
end

function tc = cross_time(t, s, level)
% First time s(t) reaches level, linear interpolation. NaN if never.
i = find(s >= level, 1);
if isempty(i)
    tc = NaN;
elseif i == 1
    tc = t(1);
else
    tc = t(i-1) + (level - s(i-1)) * (t(i) - t(i-1)) / (s(i) - s(i-1));
end
end
