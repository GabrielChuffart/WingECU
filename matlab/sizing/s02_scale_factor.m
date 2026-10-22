%% s02_scale_factor.m - Torque scale factor bench / F1, motor check
% WingECU sizing, step 2.
% Input : F1-scale hinge moment (hinge_moment.m), GM4108H-120T data.
% Output: admissible window for k = T_bench / M_F1.
%   k too high -> actuator saturates.
%   k too low  -> emulated load buried in parasitic torque (cogging, bearings).
% Inertia, spring, dynamics: s03.
% Run from matlab/sizing/, whole script or per section (Ctrl+Enter).

clear; close all; clc;
here = fileparts(mfilename('fullpath'));   % script folder
if isempty(here), here = pwd; end          % fallback: section run from the editor
addpath(fullfile(here, '..', 'models'));   % wingecu_params, hinge_moment

p      = wingecu_params();
dmax   = p.bench.delta_max;                      % [rad] full opening
M_max  = hinge_moment(p.f1.v_max, 0,    p);      % [N m] F1, closed, v_max: start of opening
M_hold = hinge_moment(p.f1.v_max, dmax, p);      % [N m] F1, open, v_max: holding

%% 1. Actuator torque capability, GM4108H-120T at V_bus
% Datasheet: 1200-1800 g.cm "load torque" at 1.5 A, 20 V, conditions not detailed.
% Cross-check from no-load speed. Ideal motor: Kt = Ke = 1/KV (SI units).
KV     = p.motor.n0_rpm / p.motor.V_test;        % [rpm/V]
Kt_est = 60 / (2*pi*KV);                         % [N m/A] = Ke [V s/rad], +/-20 % depending on drive convention
Kt_ds  = mean(p.motor.T_load) / p.motor.I_rated; % [N m/A] Kt implied by the datasheet torque

% Current limit at stall (no back-EMF), star winding, SVPWM
R_ph     = p.motor.R_ll / 2;                     % [ohm] phase resistance
V_ph_max = p.bench.V_bus / sqrt(3);              % [V]   max phase voltage amplitude
I_volt   = V_ph_max / R_ph;                      % [A]   voltage-limited current
I_max    = min(I_volt, p.motor.I_rated);         % [A]   usable peak current

T_ds  = mean(p.motor.T_load);                    % [N m] conservative case: datasheet mid value
T_est = Kt_est * I_max;                          % [N m] optimistic case: Kt estimate * I_max

% Same estimate at 12 V (proposal v2 supply), for reference
I_max_12 = min(I_volt * 12/p.bench.V_bus, p.motor.I_rated);   % [A] I_volt scales with V_bus
T_est_12 = Kt_est * I_max_12;                                 % [N m]

fprintf('--- 1. Actuator torque (V_bus = %d V) ---\n', p.bench.V_bus);
fprintf('KV = %.0f rpm/V -> Kt_est = %.3f N m/A | Kt from datasheet = %.3f N m/A (x%.1f)\n', ...
        KV, Kt_est, Kt_ds, Kt_est/Kt_ds);
fprintf('I_volt = %.2f A, I_rated = %.2f A -> I_max = %.2f A\n', I_volt, p.motor.I_rated, I_max);
fprintf('T_act: datasheet %.3f N m | Kt estimate %.3f N m | Kt estimate at 12 V %.3f N m\n\n', ...
        T_ds, T_est, T_est_12);

%% 2. Upper bound k_max: actuator torque budget
% Peak emulated load = eta_aero * T_act.
% Remainder (spring, friction, acceleration, margin) checked in s03.
T_case = [T_ds, T_est];                          % [N m] conservative / optimistic
k_max  = p.bench.eta_aero * T_case / M_max;      % [-]

%% 3. Lower bound k_min: parasitic torque
% Smallest load of interest: holding torque, flap open, v_max.
% Below snr_min * T_par, C1-C3 differences are masked by friction and cogging.
k_min = p.bench.snr_min * p.bench.T_par / M_hold;        % [-]
T_req = k_min * M_max / p.bench.eta_aero;                % [N m] min T_act for a non-empty window

fprintf('--- 2-3. Scale factor window ---\n');
fprintf('k_min = 1/%.0f  (k * M_hold >= %d * T_par)\n', 1/k_min, p.bench.snr_min);
fprintf('k_max = 1/%.0f  conservative (T_act = %.3f N m)\n', 1/k_max(1), T_case(1));
fprintf('k_max = 1/%.0f  optimistic   (T_act = %.3f N m)\n', 1/k_max(2), T_case(2));
fprintf('Non-empty window requires T_act >= %.2f N m\n\n', T_req);

%% 4. Feasibility chart: k window vs actuator torque
T_act = linspace(0.05, 0.6, 200);                % [N m] actuator torque sweep
k_up  = p.bench.eta_aero * T_act / M_max;        % [-]   upper bound vs T_act
k_50  = 1/50;                                    % [-]   initial value (2026-10-19)

ok    = k_up >= k_min;                           % [-]   T_act values with a non-empty window
k_ticks = 1 ./ [2000 1000 500 300 200 100 50];   % [-]   tick positions, labelled as 1/N

figure('Name', 'Scale factor window');
h0 = fill([T_act(ok) fliplr(T_act(ok))], ...
          [k_up(ok)  k_min*ones(1, nnz(ok))], [0.85 0.93 0.85], 'EdgeColor', 'none');
hold on;
set(gca, 'YScale', 'log');
h1 = plot(T_act, k_up, 'LineWidth', 1.5);
h2 = plot(T_act([1 end]), [1 1]*k_min, 'LineWidth', 1.5);
h3 = plot(T_act([1 end]), [1 1]*k_50, 'k:', 'LineWidth', 1.5);
y_lim = [1/2000 1/40];                           % [-] display range
h4 = plot([T_ds T_ds],   y_lim, 'k--');
h5 = plot([T_est T_est], y_lim, 'k-.');
ylim(y_lim);
set(gca, 'YTick', k_ticks, 'YTickLabel', ...
    arrayfun(@(k) sprintf('1/%d', round(1/k)), k_ticks, 'UniformOutput', false));
grid on;
xlabel('Actuator torque T_{act} [N m]');
ylabel('Scale factor k = T_{bench} / M_{F1}');
title('Admissible scale factor (shaded)');
legend([h0 h1 h2 h3 h4 h5], {'Admissible', 'k_{max}: actuator saturation', ...
       'k_{min}: parasitic torque', 'k = 1/50 (initial)', 'T_{act} datasheet', ...
       'T_{act} Kt estimate'}, 'Location', 'northwest');

%% 5. Bench torque vs speed for a provisional k
k_sel = sqrt(k_min * k_max(2));                  % [-] geometric centre of the optimistic window
v_kmh = 0:5:340;                                 % [km/h]
v     = v_kmh / 3.6;                             % [m/s]
T_closed = k_sel * hinge_moment(v, 0,    p);     % [N m] bench load, closed
T_open   = k_sel * hinge_moment(v, dmax, p);     % [N m] bench load, open

figure('Name', 'Bench torque vs speed');
plot(v_kmh, T_closed, 'LineWidth', 1.5); hold on;
plot(v_kmh, T_open,   'LineWidth', 1.5);
plot(v_kmh([1 end]), [1 1]*p.bench.eta_aero*T_est, 'k-.');
plot(v_kmh([1 end]), [1 1]*p.bench.snr_min*p.bench.T_par, 'k--');
grid on;
xlabel('Speed [km/h]');
ylabel('Bench torque [N m]');
title(sprintf('Emulated load, k = 1/%.0f', 1/k_sel));
legend('Closed', 'Open', 'Aero budget (optimistic T_{act})', 'snr_{min} \times T_{par}', ...
       'Location', 'northwest');

%% 6. Summary
fprintf('--- 6. Summary ---\n');
fprintf('k = 1/50: peak bench load %.2f N m = %.1f x datasheet torque -> rejected\n', ...
        k_50*M_max, k_50*M_max/T_ds);
if k_max(1) < k_min
    fprintf('Conservative case: no window (k_max 1/%.0f < k_min 1/%.0f)\n', 1/k_max(1), 1/k_min);
end
fprintf('Optimistic case:   window 1/%.0f ... 1/%.0f, provisional k = 1/%.0f\n', ...
        1/k_min, 1/k_max(2), 1/k_sel);
fprintf('Bench load at k = 1/%.0f: %.3f N m (closed) / %.3f N m (open) at %d km/h\n', ...
        1/k_sel, k_sel*M_max, k_sel*M_hold, round(p.f1.v_max*3.6));
fprintf('Decisive M1 measurements: Kt (window upper bound), T_par (window lower bound)\n');
