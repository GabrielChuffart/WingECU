%% s03_inertia_spring.m: Inertia, return spring, torque budget
% WingECU sizing, step 3.
% Input : provisional k_scale (s02), motor model, bench flap geometry.
% Output: rotating inertia, return spring (preload, stiffness),
%         torque budget at opening, back-EMF speed limit, holding current.
% Hand calculations, constant torque. Time-domain check: s04.
% Run from matlab/sizing/, whole script or per section (Ctrl+Enter).

clear; close all; clc;
here = fileparts(mfilename('fullpath'));   % script folder
if isempty(here), here = pwd; end          % fallback: section run from the editor
addpath(fullfile(here, '..', 'models'));   % wingecu_params, hinge_moment, motor_model

p    = wingecu_params();
m    = motor_model(p);
dmax = p.bench.delta_max;                  % [rad] full stroke
k    = p.bench.k_scale;                    % [-]   provisional (s02)

%% 1. Rotating inertia about the hinge axis
% Rotor: thin ring at mean magnet radius. Flap: thin plate, CG at mid-chord.
c_b     = p.bench.flap_c;                                  % [m] bench flap chord
d_cg    = (p.bench.x_hinge - 0.5) * c_b;                   % [m] hinge to flap CG
J_rotor = p.mech.m_rotor * p.mech.r_rotor^2;               % [kg m^2] one rotor
J_flap  = p.bench.flap_m * (c_b^2/12 + d_cg^2);            % [kg m^2] plate about CG + parallel axis
J_sum   = 2*J_rotor + J_flap;                              % [kg m^2] two rotors on the same shaft
J       = p.mech.J_margin * J_sum;                         % [kg m^2] + shaft, coupling, screws

fprintf('--- 1. Inertia ---\n');
fprintf('Rotors 2 x %.2e | flap %.2e | total with margin J = %.2e kg m^2\n', ...
        J_rotor, J_flap, J);
fprintf('Rotor share: %.0f %%\n\n', 100*2*J_rotor/J_sum);

%% 2. Motor torque-speed line at V_bus
w     = linspace(0, m.omega0, 100);                % [rad/s] speed grid
T_av  = m.T_stall * (1 - w/m.omega0);              % [N m]   available torque, motoring
w_avg = dmax / p.bench.t_close;                    % [rad/s] mean speed, full stroke in t_close
w_pk  = 2 * w_avg;                                 % [rad/s] peak speed, triangular profile
T_pk  = m.T_stall * (1 - w_pk/m.omega0);           % [N m]   available torque at w_pk
T_ld  = k * hinge_moment(p.f1.v_max, 0, p);        % [N m]   emulated load, closed, v_max

fprintf('--- 2. Torque-speed line (V_bus = %d V) ---\n', p.bench.V_bus);
fprintf('T_stall = %.3f N m, omega0 = %.1f rad/s (%.0f rpm)\n', ...
        m.T_stall, m.omega0, m.omega0*60/(2*pi));
fprintf('Full stroke in %.0f ms: mean %.1f rad/s, peak %.1f rad/s -> T_avail = %.3f N m (-%.0f %%)\n\n', ...
        p.bench.t_close*1e3, w_avg, w_pk, T_pk, 100*(1 - T_pk/m.T_stall));

%% 3. Return spring sizing
% Criteria (no aero, actuator off):
%   holds closed:    T_pre >= n_hold * T_par
%   low stiffness:   T(delta_max) = ratio * T_pre
T_pre = p.spring.n_hold * p.bench.T_par;           % [N m]     preload at delta = 0
T_end = p.spring.ratio  * T_pre;                   % [N m]     torque at delta_max
k_s   = (T_end - T_pre) / dmax;                    % [N m/rad] stiffness
T_spr = @(d) T_pre + k_s*d;                        % [N m]     spring torque vs angle, towards closed

% Fail-safe closing at v = 0, actuator off: spring against friction only.
% Net torque lower bound at delta = 0 (smallest spring torque).
% Phases open (driver off): no electrical damping.
T_net_fs = T_pre - p.bench.T_par;                  % [N m]
t_fs     = sqrt(2*J*dmax / T_net_fs);              % [s] constant torque, full stroke from rest

fprintf('--- 3. Return spring ---\n');
fprintf('Preload %.3f N m, %.3f N m at %d deg -> k_s = %.4f N m/rad\n', ...
        T_pre, T_end, round(rad2deg(dmax)), k_s);
fprintf('Fail-safe closing at v = 0: %.0f ms (target %.0f ms)\n\n', t_fs*1e3, p.bench.t_fs*1e3);

%% 4. Torque budget, start of opening at v_max
T_res   = T_ld + T_spr(0) + p.bench.T_par;         % [N m]     resisting torque at delta = 0
T_acc   = m.T_stall - T_res;                       % [N m]     left for acceleration at w = 0
alpha   = T_acc / J;                               % [rad/s^2] initial angular acceleration
w_reach = sqrt(2*alpha*dmax);                      % [rad/s]   end speed if torque stayed constant
w_eq    = m.omega0 * (1 - T_res/m.T_stall);        % [rad/s]   speed where T_avail = T_res
t_open  = dmax / w_eq;                             % [s]       lower bound, stroke at w_eq

figure('Name', 'Motor torque-speed line');
plot(w, T_av, 'LineWidth', 1.5); hold on;
plot(w([1 end]), [1 1]*T_res, 'k--');
plot(w_eq, T_res, 'ko', 'MarkerFaceColor', 'k');
plot([w_pk w_pk], [0 m.T_stall], 'k:');
grid on;
xlabel('Shaft speed \omega [rad/s]');
ylabel('Torque [N m]');
title(sprintf('GM4108H-120T at %d V (Kt estimate), opening at %d km/h', ...
      p.bench.V_bus, round(p.f1.v_max*3.6)));
legend('Available torque', 'Resisting torque (load + spring + friction)', ...
       '\omega_{eq}: speed cap under load', '\omega peak, full stroke in t_{close}', ...
       'Location', 'northeast');

shares = [T_ld, T_spr(0), p.bench.T_par, T_acc] / m.T_stall;   % [-] fractions of T_stall
figure('Name', 'Torque budget at opening');
bar(100*shares);
set(gca, 'XTickLabel', {'Aero load', 'Spring', 'Friction', 'Acceleration'});
ylabel('Share of stall torque [%]');
title(sprintf('Opening at %d km/h, k = 1/%d', round(p.f1.v_max*3.6), round(1/k)));
grid on;

fprintf('--- 4. Torque budget, opening at %d km/h ---\n', round(p.f1.v_max*3.6));
fprintf('Aero %.0f %% | spring %.0f %% | friction %.0f %% | acceleration %.0f %% of T_stall\n', ...
        100*shares);
fprintf('alpha = %.0f rad/s^2 -> %.0f rad/s at full stroke if torque stayed constant\n', ...
        alpha, w_reach);
fprintf('Back-EMF caps the speed at %.1f rad/s under this load -> opening >= %.0f ms\n\n', ...
        w_eq, t_open*1e3);

%% 5. Holding open at v_max
T_hold = k * hinge_moment(p.f1.v_max, dmax, p) + T_spr(dmax);  % [N m] static, friction ignored
I_hold = T_hold / m.Kt;                                        % [A]

fprintf('--- 5. Holding open at %d km/h ---\n', round(p.f1.v_max*3.6));
fprintf('T_hold = %.3f N m (spring %.0f %%) -> I_hold = %.2f A\n', ...
        T_hold, 100*T_spr(dmax)/T_hold, I_hold);
