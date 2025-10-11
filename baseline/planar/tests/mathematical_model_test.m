%% test_mathematical_model.m
% Sanity checks and failure-mode tests for mathematical_model()

clear; clc; close all;
rng(1);

% addpath('D:\Onedrive\Projects\Drone\Matlab_Challenge\quadrotor_ftc_rl\planar\codes');

% Auto-import ../codes (works for Live Editor and .m scripts)
try
    baseDir = fileparts(matlab.desktop.editor.getActiveFilename); % Live Editor
catch
    baseDir = fileparts(mfilename('fullpath'));                    % .m script/function
end
codeDir = fullfile(baseDir, '..', 'codes');

if exist(codeDir, 'dir') ~= 7
    error('codes folder not found at: %s\nCheck your folder layout or adjust the path.', codeDir);
end
addpath(codeDir);
fprintf('Added path: %s\n', codeDir);

% --- Common params
params = struct();
params.m   = 1.0;        % kg
params.kf  = 1.0;        % N per unit command
params.km  = 0.2;        % N*m per unit command
params.Iz  = 0.05;       % kg*m^2
params.dt  = 0.02;       % s (50 Hz)
params.integrator = 'euler';

% --- Initial state [x y vx vy psi r]
s0 = [0 0 0 0 0 0];

%% 1) Basic forward integration with random bounded controls
Tsec = 5;
N = round(Tsec/params.dt);

S = zeros(N+1,6);
U = zeros(N,4);
Fx = zeros(N,1); Fy = zeros(N,1); Mpsi = zeros(N,1);
term_flags = false(N,1);
term_reason = strings(N,1);

s = s0; S(1,:) = s;

for k = 1:N
    u = rand(1,4); % already in [0,1] - as per documentation
    U(k,:) = u;

    [s, out] = mathematical_model(s, u, params);
    S(k+1,:) = s;
    Fx(k) = out.Fx; Fy(k) = out.Fy; Mpsi(k) = out.Mpsi;
    term_flags(k) = out.terminated; term_reason(k) = out.term_reason;

    if out.terminated
        fprintf('Terminated early at step %d for reason: %s\n', k, out.term_reason);
        S = S(1:k+1,:); U = U(1:k,:); Fx = Fx(1:k); Fy = Fy(1:k); Mpsi = Mpsi(1:k);
        break;
    end
end

figure('Name','Basic Integration');
subplot(2,2,1); plot(S(:,1), S(:,2), 'LineWidth',1.5); axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Planar trajectory');

subplot(2,2,2); plot((0:size(U,1)-1)*params.dt, U, 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('u_i'); title('Motor commands'); legend('u1','u2','u3','u4');

subplot(2,2,3); plot((0:size(Fx,1)-1)*params.dt, [Fx,Fy], 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('Force [N]'); title('Fx, Fy'); legend('Fx','Fy');

subplot(2,2,4); plot((0:size(Mpsi,1)-1)*params.dt, Mpsi, 'LineWidth',1.5); grid on;
xlabel('t [s]'); ylabel('M_\psi [N·m]'); title('Yaw moment');

disp('1) Basic integration complete.');

%% 2) Action clipping check
u_bad = [-0.5, 1.2, 0.0, 2.0]; % outside [0,1]
[s1, out1] = mathematical_model(s0, u_bad, params);
assert(all(out1.u_cmd >= 0 & out1.u_cmd <= 1), 'Clipping failed on u_cmd!');
fprintf('2) Clipping OK: input %s -> clipped %s\n', mat2str(u_bad,3), mat2str(out1.u_cmd,3));

%% 3) Rate limiting check
params_rate = params;
params_rate.u_rate = 0.05;        % max change per step
params_rate.last_u = [0.2 0.2 0.2 0.2];
u_jump = [1 0 1 0];               % big change request
[~, out_rate] = mathematical_model(s0, u_jump, params_rate);
% Each element should have moved by at most 0.05 from last_u
assert(max(abs(out_rate.u_applied - params_rate.last_u)) <= params_rate.u_rate + 1e-12, 'Rate limiting failed!');
fprintf('3) Rate limiting OK: last %s, cmd %s -> applied %s\n', ...
    mat2str(params_rate.last_u,2), mat2str(u_jump,2), mat2str(out_rate.u_applied,2));

%% 4) Integrator comparison (Euler vs RK2)
% Choose a constant command that yields nonzero Fx and Fy (small yaw)
u_const = [0.7 0.2 0.1 0.9];  % Fx>0, Fy>0, Mpsi≈-0.3*km
steps = 200;

% (Optional) use a larger dt to exaggerate the integrator difference
params_e = params; params_e.integrator = 'euler'; params_e.dt = 0.05;
params_r = params; params_r.integrator = 'rk2';   params_r.dt = 0.05;

% Quick sanity print of forces/moment for this input
[~, out_chk] = mathematical_model([0 0 0 0 0 0], u_const, params_e);
fprintf('Sanity: Fx=%.3f, Fy=%.3f, Mpsi=%.3f\n', out_chk.Fx, out_chk.Fy, out_chk.Mpsi);

s_e = [0 0 0 0 0 0]; s_r = s_e;
Se = zeros(steps+1,6); Sr = Se; Se(1,:)=s_e; Sr(1,:)=s_r;

for k = 1:steps
    [s_e, ~] = mathematical_model(s_e, u_const, params_e);
    [s_r, ~] = mathematical_model(s_r, u_const, params_r);
    Se(k+1,:) = s_e; Sr(k+1,:) = s_r;
end

figure('Name','Integrator Comparison');
plot(Se(:,1), Se(:,2), '-', 'LineWidth',1.5); hold on;
plot(Sr(:,1), Sr(:,2), '--', 'LineWidth',1.5); grid on; axis equal;
legend('Euler','RK2'); xlabel('x [m]'); ylabel('y [m]');
title('Euler vs RK2 trajectory (constant input)');

dxy = hypot(Se(end,1)-Sr(end,1), Se(end,2)-Sr(end,2));
fprintf('4) Integrators compared. End-point difference: %.4f m\n', dxy);

%% 5) Termination by POSITION bound
params_pos = params;
params_pos.pos_bound = [0.5, 0.5]; % tight bounds
u_push = [0, 1, 0, 1];             % positive Fx & Fy to move away

[s_traj, out_traj] = run_episode(s0, u_push, params_pos, 200);
fprintf('5) Position-bound termination: %s at t=%.2f s\n', ...
    out_traj.term_reason, out_traj.t);

figure('Name','Position Bound Termination');
plot(s_traj(:,1), s_traj(:,2), 'LineWidth',1.5); hold on; grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); title('Termination by position bounds');
xline([-params_pos.pos_bound(1) params_pos.pos_bound(1)],':'); 
yline([-params_pos.pos_bound(2) params_pos.pos_bound(2)],':');

%% 6) Termination by VELOCITY bound
params_vel = params;
params_vel.vel_bound = 0.5; % m/s
u_fast = [0, 1, 0, 1];      % creates +Fx, +Fy -> speeds up quickly

[s_traj2, out_traj2] = run_episode(s0, u_fast, params_vel, 200);
fprintf('6) Velocity-bound termination: %s at t=%.2f s (|v|=%.2f m/s)\n', ...
    out_traj2.term_reason, out_traj2.t, hypot(s_traj2(end,3), s_traj2(end,4)));

figure('Name','Velocity Bound Termination');
subplot(2,1,1); plot((0:size(s_traj2,1)-1)*params_vel.dt, s_traj2(:,1:2), 'LineWidth',1.3); grid on;
xlabel('t [s]'); ylabel('pos [m]'); legend('x','y'); title('Position');

subplot(2,1,2); vmag = hypot(s_traj2(:,3), s_traj2(:,4));
plot((0:size(s_traj2,1)-1)*params_vel.dt, vmag, 'LineWidth',1.5); grid on;
xlabel('t [s]'); ylabel('|v| [m/s]'); yline(params_vel.vel_bound,'--');
title('Speed and velocity bound');

disp('All tests executed.');

%% ---- Helper: run_episode with constant action ----
function [S, info] = run_episode(s0, u_const, params, max_steps)
    S = zeros(max_steps+1,6);
    S(1,:) = s0;
    info = struct('terminated',false,'term_reason',"",'t',0);

    s = s0;
    for k = 1:max_steps
        [s, out] = mathematical_model(s, u_const, params);
        S(k+1,:) = s;
        if out.terminated
            info.terminated  = true;
            info.term_reason = out.term_reason;
            info.t = k*params.dt;
            S = S(1:k+1,:);
            return;
        end
    end

    info.t = max_steps*params.dt;
end
