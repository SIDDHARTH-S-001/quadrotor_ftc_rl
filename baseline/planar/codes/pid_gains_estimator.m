function [G, info] = pid_gains_estimator(m, Iz, dt, kf, km, opts)
%PID_GAINS_ESTIMATOR Robust PID gain suggestion with safety checks.
%   [G, info] = pid_gains_estimator(m, Iz, dt, kf, km, opts)
%
%   REQUIRED:
%     m, Iz   : mass [kg], yaw inertia [kg*m^2]
%     dt      : controller step [s]
%     kf, km  : force & yaw-moment gains of mixer (see model)
%
%   OPTS (struct, all optional):
%     % Target dynamics (choose rise-time OR wn directly)
%     .tr_xy     desired 10–90% rise time [s] for x,y   (default 0.5)
%     .tr_yaw    desired 10–90% rise time [s] for yaw   (default 0.4)
%     .wn_xy     natural freq [rad/s] for x,y           (overrides tr_xy)
%     .wn_yaw    natural freq [rad/s] for yaw           (overrides tr_yaw)
%     .zeta_xy   damping ratio for x,y (0.7–1.0)        (default 0.9)
%     .zeta_yaw  damping ratio for yaw                   (default 0.9)
%
%     % Actuator & constraints
%     .u_min     motor min (default 0), .u_max motor max (default 1)
%     .authority_margin   fraction of available virtual input to allow
%                         at 1 m yaw=1 rad error (default 0.6)
%
%     % Integral setup
%     .enable_integral    true/false (default true)
%     .Ti_mult_xy         Ti ≈ Ti_mult_xy/wn_xy         (default 6)
%     .Ti_mult_yaw        Ti ≈ Ti_mult_yaw/wn_yaw       (default 6)
%
%   OUTPUTS:
%     G: struct of gains {Kpx,Kdx,Kix,Kpy,Kdy,Kiy,Kppsi,Kdpsi,Kipsi}
%     info: diagnostics (caps applied, warnings, wn used, limits)
%
%   NOTES:
%     - If sampling/authority checks fail, Ki is set to 0 and info explains why.

arguments
    m  (1,1) double {mustBePositive}
    Iz (1,1) double {mustBePositive}
    dt (1,1) double {mustBePositive}
    kf (1,1) double {mustBePositive}
    km (1,1) double {mustBePositive}
    opts struct = struct()
end

% ---- defaults ----
d.tr_xy    = 0.5;   d.tr_yaw   = 0.4;
d.wn_xy    = [];    d.wn_yaw   = [];
d.zeta_xy  = 0.9;   d.zeta_yaw = 0.9;
d.u_min    = 0;     d.u_max    = 1;
d.authority_margin = 0.6;
d.enable_integral  = true;
d.Ti_mult_xy = 6;   d.Ti_mult_yaw = 6;

f = fieldnames(d);
for k=1:numel(f)
    if ~isfield(opts, f{k}), opts.(f{k}) = d.(f{k}); end
end

% ---- desired wn from tr (or use provided) ----
wn_xy  = iff(~isempty(opts.wn_xy),  opts.wn_xy,  2.2/max(opts.tr_xy,  1e-3));
wn_yaw = iff(~isempty(opts.wn_yaw), opts.wn_yaw, 2.2/max(opts.tr_yaw, 1e-3));

% ---- sampling cap ----
wN = pi/dt;            % Nyquist (rad/s)
wn_cap    = 0.2*wN;    % conservative
cap_xy  = wn_xy  > wn_cap;
cap_yaw = wn_yaw > wn_cap;
wn_xy  = min(wn_xy,  wn_cap);
wn_yaw = min(wn_yaw, wn_cap);

opts.zeta_xy  = min(max(opts.zeta_xy,  0.5), 1.2);
opts.zeta_yaw = min(max(opts.zeta_yaw, 0.5), 1.2);

% ---- PD from double integrator ----
Kpx   = m  * wn_xy^2;
Kdx   = 2  * opts.zeta_xy  * m  * wn_xy;
Kpy   = m  * wn_xy^2;
Kdy   = 2  * opts.zeta_xy  * m  * wn_xy;
Kppsi = Iz * wn_yaw^2;
Kdpsi = 2  * opts.zeta_yaw * Iz * wn_yaw;

% ---- actuator authority check ----
du = max(opts.u_max - opts.u_min, 0);
Fx_max   = 2*kf*du;
Fy_max   = 2*kf*du;
Mpsi_max = 2*km*du;

ok_Fx   = (Kpx   <= opts.authority_margin * Fx_max);
ok_Fy   = (Kpy   <= opts.authority_margin * Fy_max);
ok_Mpsi = (Kppsi <= opts.authority_margin * Mpsi_max);

scaled = false;
if ~ok_Fx || ~ok_Fy
    Kp_lim = opts.authority_margin * min(Fx_max, Fy_max);
    wn_xy_new = sqrt(max(Kp_lim, 1e-12) / m);
    if wn_xy_new < wn_xy
        wn_xy = wn_xy_new;
        Kpx   = m * wn_xy^2;
        Kdx   = 2 * opts.zeta_xy * m * wn_xy;
        Kpy   = Kpx; Kdy = Kdx;
        scaled = true;
    end
end
if ~ok_Mpsi
    Kp_lim = opts.authority_margin * Mpsi_max;
    wn_yaw_new = sqrt(max(Kp_lim, 1e-12) / Iz);
    if wn_yaw_new < wn_yaw
        wn_yaw = wn_yaw_new;
        Kppsi  = Iz * wn_yaw^2;
        Kdpsi  = 2 * opts.zeta_yaw * Iz * wn_yaw;
        scaled = true;
    end
end

% ---- Integral candidates ----
Kix = 0; Kiy = 0; Kipsi = 0; Ki_reason = "enabled";
if opts.enable_integral
    Ti_xy  = max(opts.Ti_mult_xy/wn_xy,  10*dt);
    Ti_yaw = max(opts.Ti_mult_yaw/wn_yaw,10*dt);

    if cap_xy
        Ki_reason = "disabled: wn_xy near sampling limit";
    elseif cap_yaw
        Ki_reason = "disabled: wn_yaw near sampling limit";
    elseif du <= 0
        Ki_reason = "disabled: zero actuator range";
    else
        Kix   = Kpx   / Ti_xy;
        Kiy   = Kpy   / Ti_xy;
        Kipsi = Kppsi / Ti_yaw;
        if (Kix*dt >= 0.2*Kpx) || (Kiy*dt >= 0.2*Kpy) || (Kipsi*dt >= 0.2*Kppsi)
            scale = 0.2;
            Kix   = scale*Kpx/dt;
            Kiy   = scale*Kpy/dt;
            Kipsi = scale*Kppsi/dt;
            Ki_reason = "Ki scaled to respect Ki*dt < 0.2*Kp";
        end
    end
else
    Ki_reason = "disabled by option";
end

% ---- outputs ----
G = struct('Kpx',Kpx,'Kdx',Kdx,'Kix',Kix, ...
           'Kpy',Kpy,'Kdy',Kdy,'Kiy',Kiy, ...
           'Kppsi',Kppsi,'Kdpsi',Kdpsi,'Kipsi',Kipsi);

info = struct();
info.wn_xy   = wn_xy;    info.wn_yaw = wn_yaw;
info.sampling_cap_xy  = cap_xy;  info.sampling_cap_yaw = cap_yaw;
info.scaled_for_authority = scaled;
info.authority = struct('Fx_max',Fx_max,'Fy_max',Fy_max,'Mpsi_max',Mpsi_max);
info.Ki_status = Ki_reason;
end

function y = iff(cond, a, b)
if cond, y = a; else, y = b; end
end