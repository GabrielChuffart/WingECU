%% s01_hinge_moment.m - Aerodynamic hinge moment, F1 scale
% WingECU sizing, step 1.
% Output: M_h(v, delta) of the reference flap.
%   -> load profile for the emulator, torque requirement for the actuator.
% Scaling to the bench motors: s02.
% Run from matlab/sizing/, whole script or per section (Ctrl+Enter).

clear; close all; clc;
here = fileparts(mfilename('fullpath'));   % script folder
if isempty(here), here = pwd; end          % fallback: section run from the editor
addpath(fullfile(here, '..', 'models'));   % wingecu_params, hinge_moment

p    = wingecu_params();
x_h  = p.bench.x_hinge;                    % [-]   hinge position / c
dmax = p.bench.delta_max;                  % [rad] full opening

%% 1. Geometry check: slot gap at full opening
% LE rotates about the hinge on a circle of radius r = x_h*c.
% LE displacement = chord of the arc = 2*r*sin(delta/2).
% Upper bound: whole displacement assumed to open the gap.
r_le    = x_h * p.f1.c;                                   % [m] LE-hinge distance
gap_max = p.f1.gap_closed + 2*r_le*sin(dmax/2);           % [m] gap at delta_max

% Real F1 pivot (rear of the flap): angle reaching the 85 mm limit
r_f1     = p.f1.x_pivot * p.f1.c;                                 % [m]   LE-pivot distance
delta_f1 = 2*asin((p.f1.gap_max - p.f1.gap_closed) / (2*r_f1));   % [rad] arc-chord relation inverted

fprintf('--- 1. Slot gap ---\n');
fprintf('Bench hinge at %.2f c, %d deg -> gap = %.0f mm (rule: <= %.0f mm)\n', ...
        x_h, round(rad2deg(dmax)), gap_max*1e3, p.f1.gap_max*1e3);
fprintf('F1 pivot at %.2f c          -> %.0f deg reach the %.0f mm limit\n\n', ...
        p.f1.x_pivot, rad2deg(delta_f1), p.f1.gap_max*1e3);

%% 2. Hinge moment vs speed, flap closed and fully open
v_kmh = 0:5:340;                           % [km/h] speed grid
v     = v_kmh / 3.6;                       % [m/s]

M_closed = hinge_moment(v, 0,    p);       % [N m] delta = 0: max load, torque to start opening
M_open   = hinge_moment(v, dmax, p);       % [N m] delta = max: holding torque on the straight

figure('Name', 'Hinge moment vs speed');
plot(v_kmh, M_closed, 'LineWidth', 1.5); hold on;
plot(v_kmh, M_open,   'LineWidth', 1.5);
grid on;
xlabel('Speed [km/h]');
ylabel('Hinge moment [N m]  (>0 = towards closed)');
title(sprintf('F1-scale hinge moment, hinge at %.2f c', x_h));
legend('Closed (\delta = 0)', sprintf('Open (\\delta = %d deg)', round(rad2deg(dmax))), ...
       'Location', 'northwest');

fprintf('--- 2. Hinge moment vs speed (F1 scale) ---\n');
fprintf('Speed [km/h]   closed [N m]   open [N m]\n');
for vs = [150 250 340]                     % [km/h] reference speeds
    fprintf('%8d       %8.1f      %8.1f\n', vs, ...
            hinge_moment(vs/3.6, 0, p), hinge_moment(vs/3.6, dmax, p));
end
ratio_v2 = hinge_moment(300/3.6, 0, p) / hinge_moment(150/3.6, 0, p);   % [-] M ~ v^2 -> 4 expected
fprintf('Doubling the speed multiplies the moment by %.1f\n\n', ratio_v2);

%% 3. Hinge moment vs opening angle, at several speeds
delta_deg = 0:1:40;                        % [deg]  angle grid
speeds    = [150 250 340];                 % [km/h] one curve per speed

figure('Name', 'Hinge moment vs angle');
hold on;
for vs = speeds
    plot(delta_deg, hinge_moment(vs/3.6, deg2rad(delta_deg), p), 'LineWidth', 1.5);
end
grid on;
xlabel('Opening angle \delta [deg]');
ylabel('Hinge moment [N m]');
title('F1-scale hinge moment vs opening angle');
legend(arrayfun(@(s) sprintf('%d km/h', s), speeds, 'UniformOutput', false), ...
       'Location', 'northeast');

%% 4. Sensitivity: centre of pressure and hinge position
% Lever arm (x_hinge - x_cp) = most uncertain input.
% Sweep x_cp, closed flap at v_max, bench hinge vs real F1 pivot.
x_cp_range = 0.20:0.01:0.60;               % [-]   swept CP position / c
M_ours = zeros(size(x_cp_range));          % [N m] bench hinge at x_h
M_f1   = zeros(size(x_cp_range));          % [N m] hinge at F1 pivot
pp = p;                                    % local copy, p unchanged
for i = 1:numel(x_cp_range)
    pp.aero.x_cp = x_cp_range(i);
    M_ours(i) = hinge_moment(p.f1.v_max, 0, pp, x_h);
    M_f1(i)   = hinge_moment(p.f1.v_max, 0, pp, p.f1.x_pivot);
end

y_lim = [min([M_ours M_f1 0]) - 10, max([M_ours M_f1]) + 10];   % [N m] common range, includes 0
figure('Name', 'Sensitivity to centre of pressure');
h1 = plot(x_cp_range, M_ours, 'LineWidth', 1.5); hold on;
h2 = plot(x_cp_range, M_f1,   'LineWidth', 1.5);
plot([x_cp_range(1) x_cp_range(end)], [0 0], 'k-');              % M = 0: sign change
h3 = plot([p.aero.x_cp p.aero.x_cp], y_lim, 'k--');              % nominal x_cp
ylim(y_lim);
grid on;
xlabel('Centre of pressure x_{cp} [fraction of chord]');
ylabel('Hinge moment, closed, 340 km/h [N m]');
title('Sensitivity to the centre of pressure');
legend([h1 h2 h3], {sprintf('Bench hinge (%.2f c)', x_h), ...
                    sprintf('Real F1 pivot (%.2f c)', p.f1.x_pivot), ...
                    'Nominal x_{cp}'}, 'Location', 'northeast');

M_cp25 = interp1(x_cp_range, M_ours, 0.25);   % [N m] read-out at x_cp = 0.25 c
M_cp35 = interp1(x_cp_range, M_ours, 0.35);   % [N m] read-out at x_cp = 0.35 c
fprintf('--- 4. Sensitivity (closed, 340 km/h) ---\n');
fprintf('x_cp = 0.25 c -> %5.1f N m  |  x_cp = 0.35 c -> %5.1f N m\n', M_cp25, M_cp35);
fprintf('Moment is zero when x_cp = x_hinge = %.2f c, and changes sign beyond.\n\n', x_h);

%% 5. Summary
M_max  = hinge_moment(p.f1.v_max, 0,    p);   % [N m] worst case: start opening at v_max
M_hold = hinge_moment(p.f1.v_max, dmax, p);   % [N m] hold fully open at v_max
fprintf('--- 5. Summary (F1 scale, %d km/h) ---\n', round(p.f1.v_max*3.6));
fprintf('Max moment, flap closed (to overcome when opening): %.1f N m\n', M_max);
fprintf('Moment to hold the flap fully open:                 %.1f N m\n', M_hold);
