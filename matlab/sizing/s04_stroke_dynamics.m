%% s04_stroke_dynamics.m - Opening, closing and fail-safe stroke simulation
% WingECU sizing, step 4.
% Input : bench model (s03), motor torque-speed line, emulated load k*M_h.
% Output: 10-90 % stroke times, end-stop impact speed, fail-safe closing time.
% Actuator open-loop at max torque up to the end stop: fastest possible
% stroke, lower bound for the closed-loop controllers C1-C3.
% Emulated speed constant during a stroke (braking at 5 g: -18 km/h per 100 ms).
% Run from matlab/sizing/, whole script or per section (Ctrl+Enter).

clear; close all; clc;
here = fileparts(mfilename('fullpath'));   % script folder
if isempty(here), here = pwd; end          % fallback: section run from the editor
addpath(fullfile(here, '..', 'models'));   % params, hinge_moment, motor/bench models, run_stroke

p = wingecu_params();
m = motor_model(p);
b = bench_model(p);

speeds = [150 250 340];                    % [km/h] emulated speeds for the active strokes
d_deg  = rad2deg(p.bench.delta_max);       % [deg]  full stroke, for plots

%% 1. Opening, actuator at max torque
t_op = zeros(size(speeds));                % [s]     10-90 % time
w_op = zeros(size(speeds));                % [rad/s] impact speed, open end stop

figure('Name', 'Opening stroke');
hold on;
for i = 1:numel(speeds)
    r = run_stroke(speeds(i)/3.6, 'open', p, m, b);
    t_op(i) = r.t_10_90;
    w_op(i) = r.w_stop;
    plot(1e3*r.t, rad2deg(r.delta), 'LineWidth', 1.5);
end
grid on;
xlabel('Time [ms]');
ylabel('Flap angle \delta [deg]');
ylim([0 d_deg + 5]);
title('Opening, actuator at max torque');
legend(arrayfun(@(s) sprintf('%d km/h', s), speeds, 'UniformOutput', false), ...
       'Location', 'southeast');

fprintf('--- 1. Opening ---\n');
fprintf('Speed [km/h]   t 10-90 [ms]   impact [rad/s]\n');
fprintf('%8d       %8.1f       %8.1f\n', [speeds; 1e3*t_op; w_op]);
fprintf('\n');

%% 2. Active closing, actuator at max torque
% Actuator, aero load and spring all act towards closed.
t_cl = zeros(size(speeds));                % [s]     10-90 % time
w_cl = zeros(size(speeds));                % [rad/s] impact speed, closed end stop

figure('Name', 'Active closing stroke');
hold on;
for i = 1:numel(speeds)
    r = run_stroke(speeds(i)/3.6, 'close', p, m, b);
    t_cl(i) = r.t_10_90;
    w_cl(i) = r.w_stop;
    plot(1e3*r.t, rad2deg(r.delta), 'LineWidth', 1.5);
end
grid on;
xlabel('Time [ms]');
ylabel('Flap angle \delta [deg]');
ylim([0 d_deg + 5]);
title(sprintf('Active closing, target 10-90 %% < %.0f ms', 1e3*p.bench.t_close));
legend(arrayfun(@(s) sprintf('%d km/h', s), speeds, 'UniformOutput', false), ...
       'Location', 'northeast');

fprintf('--- 2. Active closing (target %.0f ms) ---\n', 1e3*p.bench.t_close);
fprintf('Speed [km/h]   t 10-90 [ms]   impact [rad/s]\n');
fprintf('%8d       %8.1f       %8.1f\n', [speeds; 1e3*t_cl; w_cl]);
fprintf('\n');

%% 3. Fail-safe closing, actuator lost
% Driver off (phases open) vs phases shorted (electrical braking).
% v = 0: no aero help, spring alone. v = v_max: aero load helps.
fs_v    = [0 340];                         % [km/h]
fs_mode = {'off', 'short'};
t_fs    = NaN(numel(fs_mode), numel(fs_v));   % [s] time to closed end stop

figure('Name', 'Fail-safe closing');
hold on;
leg = {};
for j = 1:numel(fs_mode)
    for i = 1:numel(fs_v)
        r = run_stroke(fs_v(i)/3.6, fs_mode{j}, p, m, b);
        t_fs(j, i) = r.t_stop;
        plot(1e3*r.t, rad2deg(r.delta), 'LineWidth', 1.5);
        leg{end+1} = sprintf('%s, %d km/h', fs_mode{j}, fs_v(i)); %#ok<SAGROW>
    end
end
plot(1e3*p.bench.t_fs*[1 1], [0 d_deg], 'k--');
leg{end+1} = 'Fault target';
grid on;
xlabel('Time [ms]');
ylabel('Flap angle \delta [deg]');
ylim([0 d_deg + 5]);
title('Fail-safe closing, actuator lost at full opening');
legend(leg, 'Location', 'northeast');

fprintf('--- 3. Fail-safe closing (target %.0f ms, fault detection included) ---\n', ...
        1e3*p.bench.t_fs);
for j = 1:numel(fs_mode)
    for i = 1:numel(fs_v)
        if isnan(t_fs(j, i))
            fprintf('%-5s, %3d km/h: not closed within %.0f ms\n', ...
                    fs_mode{j}, fs_v(i), 1e3*p.sim.t_end);
        else
            fprintf('%-5s, %3d km/h: closed in %5.0f ms -> %4.0f ms left for fault detection\n', ...
                    fs_mode{j}, fs_v(i), 1e3*t_fs(j, i), 1e3*(p.bench.t_fs - t_fs(j, i)));
        end
    end
end
fprintf('\n');

%% 4. Summary
fprintf('--- 4. Summary (k = 1/%d, V_bus = %d V, Kt estimate) ---\n', ...
        round(1/p.bench.k_scale), p.bench.V_bus);
fprintf('Opening 10-90 %%:        %.0f to %.0f ms\n', 1e3*min(t_op), 1e3*max(t_op));
fprintf('Active closing 10-90 %%: %.0f to %.0f ms (target %.0f ms)\n', ...
        1e3*min(t_cl), 1e3*max(t_cl), 1e3*p.bench.t_close);
fprintf('Max impact speed:       %.1f rad/s (closed stop) -> braking before the stop required\n', ...
        max(abs(w_cl)));
