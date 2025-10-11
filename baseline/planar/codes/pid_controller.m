function [u, ctrl, diag] = pid_controller(s, ref, ctrl, params)
%PID_CONTROLLER Classical PID baseline -> virtual inputs -> motor commands.
%   [u, ctrl, diag] = pid_controller(s, ref, ctrl, params)
%
%   STATE:
%       s = [x, y, vx, vy, psi, r]
%
%   REFERENCE (yaw held at zero by design, but passed in for completeness):
%       ref.x, ref.y, ref.psi
%
%   CONTROLLER PERSISTENT STATE (pass [] to initialize):
%       ctrl.ix, ctrl.iy, ctrl.ipsi  : integral states
%       ctrl.last_u                  : last applied motor command [1x4]
%
%   PARAMS:
%       params.dt         : timestep [s]
%       params.kf, params.km
%       % PID gains:
%       params.Kpx, params.Kix, params.Kdx
%       params.Kpy, params.Kiy, params.Kdy
%       params.Kppsi, params.Kipsi, params.Kdpsi
%       % Anti-windup clamp (optional; scalar or [1x3] for x,y,psi):
%       params.Imax = [Ix Iy Ipsi] or scalar (default: Inf)
%       % Motor command limits and rate-limit (optional):
%       params.u_min  (default 0), params.u_max (default 1)
%       params.u_rate per-step |Δu|∞ elementwise limit (default: [])
%
%   OUTPUTS:
%       u     : motor commands [1x4], clipped/rate-limited
%       ctrl  : updated controller state (integrators, last_u)
%       diag  : struct with errors, virtual inputs, and pre-clip motor cmd
%
%   NOTES:
%     - Derivative action uses velocity/yaw-rate directly: ex_dot ≈ -vx, etc.
%     - Mapping uses fixed pseudo-inverse of the high-level mixer.
%     - For stability: begin with PD (Ki = 0), then add small Ki with Imax.

arguments
    s (1,6) double
    ref struct
    ctrl struct
    params struct
end

% --- unpack state ---
x   = s(1);  y  = s(2);
vx  = s(3);  vy = s(4);
psi = s(5);  r  = s(6);

% --- defaults / validation ---
req = {'dt','kf','km','Kpx','Kix','Kdx','Kpy','Kiy','Kdy','Kppsi','Kipsi','Kdpsi'};
for k=1:numel(req)
    if ~isfield(params, req{k})
        error('params.%s is required.', req{k});
    end
end

if ~isfield(params,'Imax'),   params.Imax   = inf; end
if ~isfield(params,'u_min'),  params.u_min  = 0;   end
if ~isfield(params,'u_max'),  params.u_max  = 1;   end
if ~isfield(params,'u_rate'), params.u_rate = [];  end

dt = params.dt; kf = params.kf; km = params.km;

% --- initialize controller state if needed ---
if ~isfield(ctrl,'ix'),    ctrl.ix    = 0; end
if ~isfield(ctrl,'iy'),    ctrl.iy    = 0; end
if ~isfield(ctrl,'ipsi'),  ctrl.ipsi  = 0; end
if ~isfield(ctrl,'last_u') || isempty(ctrl.last_u), ctrl.last_u = 0.5*ones(1,4); end

% --- errors (position & yaw) ---
ex   = ref.x   - x;
ey   = ref.y   - y;
epsi = ref.psi - psi;   % typically ref.psi = 0

% Derivative terms (use measured rates directly)
dex   = -vx;
dey   = -vy;
depsi = -r;

% --- integral with anti-windup clamp ---
Imax = params.Imax;
if isscalar(Imax), Imax = [Imax Imax Imax]; end

ctrl.ix   = sat(ctrl.ix   + ex*dt,   -Imax(1), Imax(1));
ctrl.iy   = sat(ctrl.iy   + ey*dt,   -Imax(2), Imax(2));
ctrl.ipsi = sat(ctrl.ipsi + epsi*dt, -Imax(3), Imax(3));

% --- PID: virtual inputs (Fx*, Fy*, Mpsi*) ---
Fx_star   = params.Kpx*ex   + params.Kdx*dex   + params.Kix*ctrl.ix;
Fy_star   = params.Kpy*ey   + params.Kdy*dey   + params.Kiy*ctrl.iy;
Mpsi_star = params.Kppsi*epsi + params.Kdpsi*depsi + params.Kipsi*ctrl.ipsi;

% --- Mixer mapping: [Fx; Fy; Mpsi] = A * u  =>  u = pinv(A) * v ---
A = [ -kf,  +kf, -kf, +kf;
      -kf,  -kf, +kf, +kf;
      +km,  -km, +km, -km ];

v = [Fx_star; Fy_star; Mpsi_star];
u_pre = pinv(A) * v;            % least-squares solution
u_pre = u_pre(:)';              % row vector [1x4]

% --- clip to [u_min, u_max] ---
u_min = params.u_min; u_max = params.u_max;
u_clip = min(max(u_pre, u_min), u_max);

% --- per-element rate limit relative to last_u (optional) ---
u = u_clip;
if ~isempty(params.u_rate) && params.u_rate > 0
    du = u - ctrl.last_u;
    du = max(min(du, params.u_rate), -params.u_rate);
    u  = ctrl.last_u + du;
    % re-enforce box
    u  = min(max(u, u_min), u_max);
end

% --- update persistent ---
ctrl.last_u = u;

% --- diagnostics ---
diag = struct();
diag.ex = ex; diag.ey = ey; diag.epsi = epsi;
diag.dex = dex; diag.dey = dey; diag.depsi = depsi;
diag.Fx_star = Fx_star; diag.Fy_star = Fy_star; diag.Mpsi_star = Mpsi_star;
diag.u_pre = u_pre; diag.u_clip = u_clip;

end

% ---------- helpers ----------
function y = sat(x, lo, hi)
    y = min(max(x, lo), hi);
end
