function test_baseline_waypoints()
%% TEST BASELINE: sanity sims + failure-mode checks for baseline_waypoint

clc; close all; rng(0);

%% ---------------------- Common plant params ----------------------
params = struct();
params.m   = 1.0;       % kg
params.kf  = 5.0;       % N per unit command (simple high-level gain)
params.km  = 0.2;       % N*m per unit command
params.Iz  = 0.05;      % kg*m^2
params.dt  = 0.02;      % 50 Hz
params.integrator = 'euler';

% Safety
params.u_rate   = 0.08;          % per-step max |Δu_i|
params.pos_bound= [50, 50];      % terminate if |x|>50 or |y|>50
params.vel_bound= 25;            % terminate if speed > 25 m/s

% Mixer matrix (consistent with your model)
A = [ -params.kf,  +params.kf,  -params.kf,  +params.kf;
      -params.kf,  -params.kf,  +params.kf,  +params.kf;
      +params.km,  -params.km,  +params.km,  -params.km ];
A_dag = pinv(A);  % pseudoinverse for mapping virtual [Fx;Fy;Mpsi] -> u

% Simple PD gains (no I for this test)
Kpx = 3.0;  Kdx = 2.0;
Kpy = 3.0;  Kdy = 2.0;
Kppsi = 2.0; Kdpsi = 0.3;

%% ---------------------- SIM 1: Parametric mode ----------------------
disp('--- SIM 1: Parametric straight-line at constant speed ---');

cfg1 = struct( ...
    'P0', [0 0], ...
    'P1', [15 5], ...
    'mode','param', ...
    'v_ref', 1.5 ...
);

Tsim = 20;                 % s
N    = round(Tsim/params.dt);

s = [0 0 0 0 0 0];         % [x y vx vy psi r]
last_u = 0.5*ones(1,4);    % start mid-throttle (no vertical in this model, but helps rate-limit)
params.last_u = last_u;

% Logs
log1.t    = zeros(N,1);
log1.s    = zeros(N,6);
log1.ref  = zeros(N,3);   % [x*, y*, psi*]
log1.u    = zeros(N,4);
log1.F    = zeros(N,3);   % [Fx, Fy, Mpsi]
done_flag = false;

for k = 1:N
    t = (k-1)*params.dt;

    % Reference (param mode ignores pos input; call with 3 args)
    [ref, ~] = baseline_waypoint(t, cfg1, struct());
    xref = ref.x; yref = ref.y; psiref = ref.psi;

    % Errors
    ex = xref - s(1);
    ey = yref - s(2);
    epsi = psiref - s(5);

    % PD virtual controls
    Fx_des   = Kpx*ex + Kdx*( - s(3) );
    Fy_des   = Kpy*ey + Kdy*( - s(4) );
    Mpsi_des = Kppsi*epsi + Kdpsi*( - s(6) );

    % Map to motors via pseudo-inverse, then clip in plant
    u_cmd = (A_dag * [Fx_des; Fy_des; Mpsi_des])'; % row 1x4

    % Plant step
    params.last_u = last_u;
    [s_next, out] = mathematical_model(s, u_cmd, params);
    last_u = out.u_applied;  % for rate limit continuity

    % Logs
    log1.t(k)    = t;
    log1.s(k,:)  = s;
    log1.ref(k,:) = [xref, yref, psiref];
    log1.u(k,:)  = out.u_applied;
    log1.F(k,:)  = [out.Fx, out.Fy, out.Mpsi];

    s = s_next;

    if out.terminated && ~done_flag
        fprintf('SIM1 terminated early at t=%.2f s due to %s\n', t, out.term_reason);
        done_flag = true;
        % (continue to fill remaining logs with last state if you prefer)
        break;
    end
end

% Trim logs if early termination
if done_flag
    idx = 1:k;
else
    idx = 1:N;
end

figure('Name','SIM1 Parametric');
subplot(2,2,1); hold on; axis equal; grid on;
plot(log1.s(idx,1), log1.s(idx,2), 'LineWidth',1.5);
plot([cfg1.P0(1) cfg1.P1(1)], [cfg1.P0(2) cfg1.P1(2)], '--');
legend('Trajectory','Line','Location','best');
xlabel('x [m]'); ylabel('y [m]'); title('Path');

subplot(2,2,2); grid on;
plot(log1.t(idx), log1.s(idx,5), 'LineWidth',1.2); hold on;
yline(0,'--');
xlabel('t [s]'); ylabel('\psi [rad]'); title('Yaw');

subplot(2,2,3); grid on;
plot(log1.t(idx), log1.u(idx,:),'LineWidth',1.0);
xlabel('t [s]'); ylabel('u_i'); title('Motor commands'); ylim([-0.1 1.1]);
legend('u1','u2','u3','u4');

subplot(2,2,4); grid on;
plot(log1.t(idx), vecnorm(log1.s(idx,3:4),2,2), 'LineWidth',1.2);
xlabel('t [s]'); ylabel('speed [m/s]'); title('Ground speed');

%% ---------------------- SIM 2: Waypoint-list mode ----------------------
disp('--- SIM 2: Discrete waypoints with radius advancement ---');

cfg2 = struct( ...
    'P0', [0 0], ...
    'P1', [15 5], ...
    'mode','waypoints', ...
    'N', 31, ...
    'radius', 0.25 ...
);

Tsim2 = 25; N2 = round(Tsim2/params.dt);
s = [0 0 0 0 0 0];
last_u = 0.5*ones(1,4);
params.last_u = last_u;
ctx = struct();   % must be struct (not []), due to arguments validation

log2.t    = zeros(N2,1);
log2.s    = zeros(N2,6);
log2.ref  = zeros(N2,3);
log2.u    = zeros(N2,4);
log2.idx  = zeros(N2,1);
done_flag = false;

for k = 1:N2
    t = (k-1)*params.dt;

    % Here waypoint mode REQUIRES pos to advance indices
    pos_now = s(1:2);
    [ref, ctx] = baseline_waypoint(t, cfg2, ctx, pos_now);
    xref = ref.x; yref = ref.y; psiref = ref.psi;

    % PD virtual controls
    ex = xref - s(1);
    ey = yref - s(2);
    epsi = psiref - s(5);

    Fx_des   = Kpx*ex + Kdx*( - s(3) );
    Fy_des   = Kpy*ey + Kdy*( - s(4) );
    Mpsi_des = Kppsi*epsi + Kdpsi*( - s(6) );

    u_cmd = (A_dag * [Fx_des; Fy_des; Mpsi_des])';

    params.last_u = last_u;
    [s_next, out] = mathematical_model(s, u_cmd, params);
    last_u = out.u_applied;

    % Logs
    log2.t(k)    = t;
    log2.s(k,:)  = s;
    log2.ref(k,:) = [xref, yref, psiref];
    log2.u(k,:)  = out.u_applied;
    log2.idx(k)  = ref.i;

    s = s_next;

    if ref.done && ~done_flag
        fprintf('SIM2: final waypoint reached around t=%.2f s (i=%d)\n', t, ref.i);
        done_flag = true;
        % keep running to see hold behavior near final waypoint
    end

    if out.terminated
        fprintf('SIM2 terminated early at t=%.2f s due to %s\n', t, out.term_reason);
        break;
    end
end

idx2 = 1:k;

figure('Name','SIM2 Waypoints');
subplot(2,2,1); hold on; axis equal; grid on;
plot(log2.s(idx2,1), log2.s(idx2,2), 'LineWidth',1.5);
plot([cfg2.P0(1) cfg2.P1(1)], [cfg2.P0(2) cfg2.P1(2)], '--');
scatter(linspace(cfg2.P0(1),cfg2.P1(1),cfg2.N), linspace(cfg2.P0(2),cfg2.P1(2),cfg2.N), 12, 'filled');
legend('Trajectory','Line','Waypoints','Location','best');
xlabel('x [m]'); ylabel('y [m]'); title('Path & waypoints');

subplot(2,2,2); grid on;
plot(log2.t(idx2), log2.idx(idx2), 'LineWidth',1.2);
xlabel('t [s]'); ylabel('waypoint index i'); title('Waypoint advancement');

subplot(2,2,3); grid on;
plot(log2.t(idx2), log2.u(idx2,:),'LineWidth',1.0);
xlabel('t [s]'); ylabel('u_i'); title('Motor commands'); ylim([-0.1 1.1]);
legend('u1','u2','u3','u4');

subplot(2,2,4); grid on;
plot(log2.t(idx2), vecnorm(log2.s(idx2,3:4),2,2), 'LineWidth',1.2);
xlabel('t [s]'); ylabel('speed [m/s]'); title('Ground speed');

%% ---------------------- FAILURE MODES (baseline_waypoint) ----------------------
disp('--- FAILURE MODE CHECKS (baseline_waypoint) ---');

% 1) Missing P0
try
    cfg = struct('P1',[1 1],'mode','param','v_ref',1);
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Missing P0');
catch ME
    reportPass('Missing P0', ME);
end

% 2) Missing P1
try
    cfg = struct('P0',[0 0],'mode','param','v_ref',1);
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Missing P1');
catch ME
    reportPass('Missing P1', ME);
end

% 3) Missing mode
try
    cfg = struct('P0',[0 0],'P1',[1 1],'v_ref',1);
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Missing mode');
catch ME
    reportPass('Missing mode', ME);
end

% 4) Unknown mode
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','banana');
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Unknown mode');
catch ME
    reportPass('Unknown mode', ME);
end

% 5) PARAM: missing v_ref
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','param');
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Param: missing v_ref');
catch ME
    reportPass('Param: missing v_ref', ME);
end

% 6) PARAM: non-positive v_ref
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','param','v_ref',0);
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Param: non-positive v_ref');
catch ME
    reportPass('Param: non-positive v_ref', ME);
end

% 7) WAYPOINTS: N < 2
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','waypoints','N',1,'radius',0.2);
    baseline_waypoint(0.0, cfg, struct(), [0 0]);
    reportFail('Waypoints: N < 2');
catch ME
    reportPass('Waypoints: N < 2', ME);
end

% 8) WAYPOINTS: missing radius
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','waypoints','N',10);
    baseline_waypoint(0.0, cfg, struct(), [0 0]);
    reportFail('Waypoints: missing radius');
catch ME
    reportPass('Waypoints: missing radius', ME);
end

% 9) WAYPOINTS: non-positive radius
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','waypoints','N',10,'radius',0);
    baseline_waypoint(0.0, cfg, struct(), [0 0]);
    reportFail('Waypoints: non-positive radius');
catch ME
    reportPass('Waypoints: non-positive radius', ME);
end

% 10) WAYPOINTS: missing pos
try
    cfg = struct('P0',[0 0],'P1',[1 1],'mode','waypoints','N',10,'radius',0.2);
    % omit pos -> should error
    baseline_waypoint(0.0, cfg, struct());
    reportFail('Waypoints: missing pos');
catch ME
    reportPass('Waypoints: missing pos', ME);
end

disp('--- FAILURE CHECKS COMPLETE ---');

end % function test_baseline

%% ---------------------- Helpers ----------------------
function reportPass(name, ME)
fprintf('[PASS] %-30s -> %s\n', name, ME.message);
end

function reportFail(name)
fprintf(2, '[FAIL] %-30s -> no error thrown\n', name); % print in red
end
