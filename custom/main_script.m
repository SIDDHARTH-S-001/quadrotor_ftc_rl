%% Main Script for Fault-Free vs. Faulty Motor Comparison
clear; clc; close all;
% Clear persistent state in helper functions
clear pid_controller kalman_filter calculate_error

% ===== Shared Parameters =====
params.m      = 1.0;      % Mass (kg)
params.g      = 9.81;     % Gravity (m/s^2)
params.Ix     = 0.01;     % Moment of inertia (X-axis)
params.Iy     = 0.01;     % Moment of inertia (Y-axis)
params.Iz     = 0.02;     % Moment of inertia (Z-axis)
params.L      = 0.2;      % Arm length (m)
params.JR     = 1e-4;     % Propeller inertia
params.OmegaR = 0;        % Nominal propeller speed
params.c_tau  = 1e-2;     % Drag‐torque coefficient
params.dist   = zeros(6,1);  % Disturbances (d1…d6)

% ===== Velocity & Acceleration Limits =====
% maximum linear velocity [vx; vy; vz] in m/s
params.v_max = [5; 5; 3];
% maximum linear acceleration [ax; ay; az] in m/s^2
params.a_max = [5; 5; 5];

% ===== PID Gains =====
gains.Kp_pos = [1.5, 1.5, 2.0];
gains.Ki_pos = [0.1, 0.1, 0.2];
gains.Kd_pos = [0.5, 0.5, 0.8];
gains.Kp_att = [8.0, 8.0, 5.0];
gains.Ki_att = [0.5, 0.5, 0.3];
gains.Kd_att = [2.0, 2.0, 1.5];

% ===== Kalman Filter Setup =====
dt = 0.01;
A  = eye(12);
H  = eye(12);
Q  = diag(0.01 * ones(12,1));
R  = diag(0.1  * ones(12,1));

% ===== Time Vector =====
t = 0:dt:10;

% ===== Generate Reference Trajectory =====
ref = generate_reference(t);

% ===== Initialize State =====
x0 = [ ref.px(1);
       ref.py(1);
       ref.pz(1);
       ref.vx(1);
       ref.vy(1);
       ref.vz(1);
       0; 0; ref.yaw(1);
       0; 0; 0 ];

% ===== Simulation Cases =====
cases = {'Fault-Free','Faulty Motor'};
results = struct();

for case_idx = 1:2
    % Reset PID integrator once per case
    clear pid_controller

    % Initialize states and filter
    x_true   = x0;
    x_est    = x0;
    P        = eye(12);
    fault_active    = false;
    last_fault_time = -Inf;

    x_history     = zeros(numel(t), 12);
    error_history = zeros(numel(t), 6);
    motor_thrusts = zeros(numel(t), 4);

    for i = 1:numel(t)
        % --- Sensor measurement w/ noise ---
        y = x_true + sqrt(R)*randn(12,1);

        % --- Kalman Filter: predict & update ---
        [x_est, P] = kalman_filter(y, x_est, P, A, H, Q, R);

        % --- PID Controller ---
        current_ref.px  = ref.px(i);
        current_ref.py  = ref.py(i);
        current_ref.pz  = ref.pz(i);
        current_ref.yaw = ref.yaw(i);
        u = pid_controller(current_ref, x_est, params, gains, dt);

        % --- Simulate Motor Fault (case 2 only) ---
        if case_idx == 2
            [u, fault_active, last_fault_time] = simulate_motor_fault( ...
                u, t(i), fault_active, last_fault_time);
        end
        motor_thrusts(i,:) = u';

        % --- Propagate True State with RK4 ---
        f = @(xx) quadrotor_model(0, xx, u, params);
        k1 = f(x_true);
        k2 = f(x_true + 0.5*dt*k1);
        k3 = f(x_true + 0.5*dt*k2);
        k4 = f(x_true +     dt*k3);
        x_true = x_true + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

        % --- Enforce ground: z >= 0, no downward vel on ground ---
        if x_true(3) < 0
            x_true(3) = 0;
            x_true(6) = max(0, x_true(6));
        end

        x_history(i,:) = x_true';

        % --- Log Tracking Errors via helper ---
        err = calculate_error(ref, x_true, i);
        error_history(i,:) = [err.position; err.attitude]';
    end

    results(case_idx).x_history     = x_history;
    results(case_idx).error_history = error_history;
    results(case_idx).motor_thrusts = motor_thrusts;
end

% ===== Plot Trajectories =====
figure('Name','Trajectory Comparison','NumberTitle','off');
plot3(ref.px, ref.py, ref.pz, 'k-', 'LineWidth', 2); hold on;
plot3(results(1).x_history(:,1), results(1).x_history(:,2), results(1).x_history(:,3), 'b-');
plot3(results(2).x_history(:,1), results(2).x_history(:,2), results(2).x_history(:,3), 'r--');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Quadrotor Trajectory Comparison');
legend('Reference','Fault-Free','Faulty Motor','Location','best');
grid on; axis equal; view(45,30);

% Zoom into the reference envelope
margin = 0.5;
xlim([min(ref.px)-margin, max(ref.px)+margin]);
ylim([min(ref.py)-margin, max(ref.py)+margin]);
zlim([min(ref.pz)-margin, max(ref.pz)+margin]);

% ===== Plot Performance Metrics =====
figure('Name','Performance Metrics','NumberTitle','off');
% Position Errors
subplot(2,1,1);
plot(t, results(1).error_history(:,1:3), 'b-'); hold on;
plot(t, results(2).error_history(:,1:3), 'r--');
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Tracking Errors');
legend('X_{ff}','Y_{ff}','Z_{ff}', 'X_{faulty}','Y_{faulty}','Z_{faulty}', 'Location','best');
grid on;
% Motor Thrusts (Faulty Only)
subplot(2,1,2);
plot(t, results(2).motor_thrusts, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Motor Thrusts (Faulty Case)');
legend('M1','M2','M3','M4','Location','best');
grid on;
