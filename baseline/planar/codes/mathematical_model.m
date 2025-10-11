function [s_next, out] = mathematical_model(s, u, params)
%MATHEMATICAL_MODEL Planar (x,y) + yaw quad model with simple mixer.
%   Discrete-time update using forward Euler (default) or RK2.
%
%   STATE:
%       s = [x, y, vx, vy, psi, r]
%
%   ACTION (motor commands, high level):
%       u = [u1, u2, u3, u4]'  with each in [0,1]
%       (1: front-left, 2: front-right, 3: rear-right, 4: rear-left)
%
%   MIXER (decoupled from yaw):
%       Fx   = kf * ((u2 + u4) - (u1 + u3))
%       Fy   = kf * ((u3 + u4) - (u1 + u2))
%       Mpsi = km * ( u1 - u2 +  u3 -  u4)
%
%   DYNAMICS:
%       xdot  = vx
%       ydot  = vy
%       vxdot = Fx / m
%       vydot = Fy / m
%       psidot= r
%       rdot  = Mpsi / Iz
%
%   INPUTS:
%       params.m        (kg)          mass
%       params.kf       (N)           force gain per unit motor command
%       params.km       (N*m)         yaw moment gain per unit motor command
%       params.Iz       (kg*m^2)      yaw inertia
%       params.dt       (s)           timestep
%       params.integrator 'euler'|'rk2' (optional, default 'euler')
%
%   SAFETY/LIMITS (all optional):
%       params.u_rate   max |Δu| per step (0..1). If provided with params.last_u, rate limit is applied.
%       params.last_u   previous applied motor command (1x4)
%       params.pos_bound [rx, ry] terminate if |x|>rx or |y|>ry
%       params.vel_bound vmax terminate if hypot(vx,vy)>vmax
%
%   OUTPUTS:
%       s_next : next state (1x6)
%       out    : struct with diagnostics
%           .u_cmd      commanded u (clipped to [0,1])
%           .u_applied  after rate limit and clipping
%           .Fx, .Fy, .Mpsi
%           .terminated (logical)
%           .term_reason (string)
%
%   NOTE: This function implements ONLY the plant; no faults or controllers.

% ----------- defaults & validation -----------
arguments
    s (1,6) double
    u (1,4) double
    params struct
end

% Required params
req = {'m','kf','km','Iz','dt'};
for k = 1:numel(req)
    if ~isfield(params, req{k})
        error('params.%s is required.', req{k});
    end
end

% Optional params with defaults
if ~isfield(params, 'integrator'), params.integrator = 'euler'; end
if ~isfield(params, 'u_rate'),     params.u_rate = [];          end
if ~isfield(params, 'last_u'),     params.last_u = [];          end
if ~isfield(params, 'pos_bound'),  params.pos_bound = [];       end
if ~isfield(params, 'vel_bound'),  params.vel_bound = [];       end

% ----------- clip & rate-limit action -----------
u_cmd = min(max(u, 0), 1); % box clip [0,1]

u_applied = u_cmd;
if ~isempty(params.u_rate) && ~isempty(params.last_u)
    du = u_cmd - params.last_u;
    du = max(min(du, params.u_rate), -params.u_rate); % ∞-norm per element
    u_applied = params.last_u + du;
    % ensure we still respect [0,1]
    u_applied = min(max(u_applied, 0), 1);
end

% ----------- mixer -----------
kf   = params.kf;
km   = params.km;

u1 = u_applied(1); u2 = u_applied(2); u3 = u_applied(3); u4 = u_applied(4);

Fx   = kf * ((u2 + u4) - (u1 + u3));
Fy   = kf * ((u3 + u4) - (u1 + u2));
Mpsi = km * (  u1 - u2 +  u3 -  u4);

% ----------- continuous dynamics -----------
m  = params.m;
Iz = params.Iz;

x   = s(1);  y  = s(2);
vx  = s(3);  vy = s(4);
psi = s(5);  r  = s(6);

xdot   = vx;
ydot   = vy;
vxdot  = Fx / m;
vydot  = Fy / m;
psidot = r;
rdot   = Mpsi / Iz;

dt = params.dt;

% ----------- integrate -----------
switch lower(string(params.integrator))
    case "rk2" % midpoint method
        % k1
        k1 = [xdot, ydot, vxdot, vydot, psidot, rdot];

        % state at midpoint using k1
        s_mid = s + 0.5*dt*k1;

        % recompute dynamics at midpoint (forces constant over dt)
        xdot2   = s_mid(3);
        ydot2   = s_mid(4);
        vxdot2  = Fx / m;
        vydot2  = Fy / m;
        psidot2 = s_mid(6);
        rdot2   = Mpsi / Iz;

        k2 = [xdot2, ydot2, vxdot2, vydot2, psidot2, rdot2];

        s_next = s + dt * k2;

    otherwise % 'euler'
        s_next = s + dt * [xdot, ydot, vxdot, vydot, psidot, rdot];
end

% ----------- termination checks -----------
terminated  = false;
term_reason = "";

if ~isempty(params.pos_bound)
    rx = params.pos_bound(1);
    ry = params.pos_bound(2);
    if abs(s_next(1)) > rx || abs(s_next(2)) > ry
        terminated  = true;
        term_reason = "position_bound";
    end
end

if ~terminated && ~isempty(params.vel_bound)
    if hypot(s_next(3), s_next(4)) > params.vel_bound
        terminated  = true;
        term_reason = "velocity_bound";
    end
end

% ----------- outputs -----------
out = struct();
out.u_cmd      = u_cmd;
out.u_applied  = u_applied;
out.Fx         = Fx;
out.Fy         = Fy;
out.Mpsi       = Mpsi;
out.terminated = terminated;
out.term_reason= term_reason;

end
