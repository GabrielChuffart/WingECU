function b = bench_model(p)
%BENCH_MODEL  Mechanical parameters of the bench, derived from wingecu_params.
%   b = bench_model(p) returns:
%     J_rotor, J_flap, J  [kg m^2]   inertia about the hinge axis
%     T_pre, T_end        [N m]      spring torque at delta = 0 and delta_max
%     k_s                 [N m/rad]  spring stiffness
%   Derivation and criteria: s03.

c_b       = p.bench.flap_c;                                  % [m] bench flap chord
d_cg      = (p.bench.x_hinge - 0.5) * c_b;                   % [m] hinge to flap CG (mid-chord)
b.J_rotor = p.mech.m_rotor * p.mech.r_rotor^2;               % [kg m^2] one rotor, thin ring
b.J_flap  = p.bench.flap_m * (c_b^2/12 + d_cg^2);            % [kg m^2] plate about CG + parallel axis
b.J       = p.mech.J_margin * (2*b.J_rotor + b.J_flap);      % [kg m^2] two rotors + flap + margin

b.T_pre   = p.spring.n_hold * p.bench.T_par;                 % [N m]     preload, holds closed without aero
b.T_end   = p.spring.ratio  * b.T_pre;                       % [N m]     torque at delta_max
b.k_s     = (b.T_end - b.T_pre) / p.bench.delta_max;         % [N m/rad] stiffness
end
