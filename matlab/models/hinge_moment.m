function M = hinge_moment(v, delta, p, x_hinge)
%HINGE_MOMENT  Aerodynamic moment of the reference flap about its hinge.
%   M = hinge_moment(v, delta, p)           hinge at p.bench.x_hinge
%   M = hinge_moment(v, delta, p, x_hinge)  hinge at x_hinge (fraction of c)
%
%   v      [m/s]  speed (scalar or array)
%   delta  [rad]  opening angle, 0 = closed (high downforce)
%   M      [N m]  hinge moment, > 0 towards closed
%
%   Model (proposal, eq. 1):
%       M  = q * S * c * Ch(delta)
%       Ch = CN(delta) * (x_hinge - x_cp)
%   Normal force CN*q*S applied at the centre of pressure x_cp,
%   lever arm (x_hinge - x_cp)*c. CP ahead of hinge -> M > 0 (fail-safe).

if nargin < 4
    x_hinge = p.bench.x_hinge;
end

q  = 0.5 * p.rho * v.^2;              % [Pa]   dynamic pressure
S  = p.f1.b * p.f1.c;                 % [m^2]  flap planform area
CN = flap_CN(delta, p);               % [-]    normal-force coeff. at delta
Ch = CN .* (x_hinge - p.aero.x_cp);   % [-]    hinge-moment coeff. = CN * (lever arm / c)
M  = q .* S .* p.f1.c .* Ch;          % [N m]  q*S -> force scale, *c -> moment scale
end

function CN = flap_CN(delta, p)
%FLAP_CN  Normal-force coefficient vs opening angle (linear, first assumption).
s  = min(max(delta / p.bench.delta_max, 0), 1);                    % [-] normalised opening, clamped to [0, 1]
CN = p.aero.CN_closed + (p.aero.CN_open - p.aero.CN_closed) .* s;   % [-] linear interp. closed -> open
end
