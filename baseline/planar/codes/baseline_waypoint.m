function [ref, ctx] = baseline_waypoint(t, cfg, ctx, pos)
% BASELINE_WAYPOINT Straight-line 2D reference generator with two modes.
%   Supports:
%     1) Parametric at constant ground speed v_ref
%     2) Discrete waypoint list with radius-based advancement
%
%   [ref, ctx] = baseline_waypoint(t, cfg, ctx, pos)
%
%   INPUTS
%     t    : current time [s]
%     cfg  : configuration struct with fields
%            .P0        [1x2] start point [x0,y0]
%            .P1        [1x2] end point   [x1,y1]
%            .mode      'param' | 'waypoints'
%            .v_ref     (param mode) ground speed [m/s]
%            .N         (waypoints mode) number of samples along the line (>=2)
%            .radius    (waypoints mode) acceptance radius to advance [m]
%     ctx  : (waypoints mode) persistent state struct; pass [] to init
%            fields used/returned:
%              .wps  [Nx2] pre-sampled waypoints
%              .i    current waypoint index (1..N)
%              .done logical, reached final waypoint
%     pos  : current position [x,y] (required for 'waypoints' mode)
%
%   OUTPUTS
%     ref : struct with reference signals
%           .x, .y     position reference
%           .psi       yaw reference (always 0)
%           .done      logical, finished path
%           .mode      echo of cfg.mode
%           .i         (waypoints) current waypoint index
%     ctx : updated context (return and pass back in next call)
%
%   NOTES
%     - Yaw reference is held at zero: psi*(t) = 0.
%     - For 'param' mode, pos input is ignored.
%     - For 'waypoints' mode, pass pos each call to allow index advancement.
%
%   EXAMPLE (param):
%     cfg = struct('P0',[0 0],'P1',[10 0],'mode','param','v_ref',1.0);
%     [ref,~] = baseline_waypoint(3.5, cfg, [], []);
%
%   EXAMPLE (waypoints):
%     cfg = struct('P0',[0 0],'P1',[10 0],'mode','waypoints','N',21,'radius',0.2);
%     ctx = []; pos = [0 0];
%     for k=1:100
%         [ref, ctx] = baseline_waypoint(0.02*k, cfg, ctx, pos);
%         % ... your controller moves 'pos' toward [ref.x, ref.y] ...
%     end

arguments
    t (1,1) double
    cfg struct
    ctx struct = struct()
    pos (1,2) double = [NaN NaN]
end

% ---- Validate & defaults ----
mustHave = {'P0','P1','mode'};
for k = 1:numel(mustHave)
    if ~isfield(cfg, mustHave{k})
        error('cfg.%s is required.', mustHave{k});
    end
end

P0 = cfg.P0(:)'; % [1x2]
P1 = cfg.P1(:)';

d  = P1 - P0;
L  = hypot(d(1), d(2));
if L == 0
    warning('P0 and P1 are identical; using stationary reference at P0.');
    d_hat = [0 0];
else
    d_hat = d / L;
end

mode = lower(string(cfg.mode));
ref = struct('x', P0(1), 'y', P0(2), 'psi', 0.0, 'done', false, 'mode', char(mode), 'i', NaN);

switch mode
    case "param"
        if ~isfield(cfg, 'v_ref') || cfg.v_ref <= 0
            error('cfg.v_ref > 0 is required for mode="param".');
        end
        s = min(cfg.v_ref * max(t,0), L);  % arc-length traveled along the line
        p = P0 + d_hat * s;
        ref.x   = p(1);
        ref.y   = p(2);
        ref.psi = 0.0;
        ref.done= (s >= L - 1e-9);

    case "waypoints"
        if ~isfield(cfg,'N') || cfg.N < 2
            error('cfg.N >= 2 is required for mode="waypoints".');
        end
        if ~isfield(cfg,'radius') || cfg.radius <= 0
            error('cfg.radius > 0 is required for mode="waypoints".');
        end
        if any(isnan(pos))
            error('pos [x,y] must be provided for mode="waypoints".');
        end

        % Initialize context if empty
        if ~isfield(ctx,'wps') || isempty(ctx)
            xs = linspace(P0(1), P1(1), cfg.N);
            ys = linspace(P0(2), P1(2), cfg.N);
            ctx.wps  = [xs(:), ys(:)];
            ctx.i    = 1;
            ctx.done = false;
        end

        % Current target waypoint
        target = ctx.wps(ctx.i, :);

        % Advance if within radius
        if ~ctx.done
            if norm(pos - target) <= cfg.radius
                ctx.i = min(ctx.i + 1, size(ctx.wps,1));
                target = ctx.wps(ctx.i, :);
            end
            if ctx.i == size(ctx.wps,1) && norm(pos - target) <= cfg.radius
                ctx.done = true;
            end
        end

        % Output current target as reference
        ref.x    = target(1);
        ref.y    = target(2);
        ref.psi  = 0.0;
        ref.done = ctx.done;
        ref.i    = ctx.i;

    otherwise
        error('Unknown cfg.mode "%s". Use "param" or "waypoints".', cfg.mode);
end

end
