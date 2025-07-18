%% Main Script for Fault-Free vs. Faulty Motor Comparison
clear; clc; close all;

% ===== Shared Parameters =====
params.m = 1.0;     % Mass (kg)
params.g = 9.81;    % Gravity (m/s²)
params.Ix = 0.01;   % Moment of inertia (X-axis)
params.Iy = 0.01;   % Moment of inertia (Y-axis)
params.Iz = 0.02;   % Moment of inertia (Z-axis)
params.L = 0.2;     % Arm length (m)
params.JR = 1e-4;   % Propeller inertia
params.OmegaR = 0;  % Propeller angular velocity
params.d = zeros(6,1); % Disturbances (zero for now)

% ===== PID Gains =====
gains.Kp_pos = [1.5, 1.5, 2.0];
gains.Ki_pos = [0.1, 0.1, 0.2];
gains.Kd_pos = [0.5, 0.5, 0.8];
gains.Kp_att = [8.0, 8.0, 5.0];
gains.Ki_att = [0.5, 0.5, 0.3];
gains.Kd_att = [2.0, 2.0, 1.5];

% ===== Kalman Filter Setup =====
dt = 0.01;
A = eye(12);
H = eye(12);
Q = diag(0.01 * ones(12,1));
R = diag(0.1 * ones(12,1));

% ===== Time Vector =====
t = 0:dt:40;

% ===== Generate Reference Trajectory =====
ref.px = sin(pi * t / 500);
ref.py = sin(pi * t / 500);
ref.pz = ones(size(t));
ref.vx = (pi/500) * cos(pi * t / 500);
ref.vy = (pi/500) * cos(pi * t / 500);
ref.vz = zeros(size(t));
ref.yaw = zeros(size(t));

% ===== Initialize State =====
x0 = [ref.px(1); ref.py(1); ref.pz(1); ...
      ref.vx(1); ref.vy(1); ref.vz(1); ...
      0; 0; ref.yaw(1); 0; 0; 0];

% ===== Simulation Cases =====
cases = {'Fault-Free', 'Faulty Motor'};
results = struct();

for case_idx = 1:2
    x_true = x0;
    x_est = x0;
    P = eye(12);
    fault_active = false;
    last_fault_time = -Inf;
    
    x_history = zeros(length(t), 12);
    error_history = zeros(length(t), 6);
    motor_thrusts = zeros(length(t), 4); % Store motor thrusts
    
    for i = 1:length(t)
        % Sensor measurements (with noise)
        y = x_true + sqrt(R) * randn(12,1);
        
        % Kalman Filter
        [x_est, P] = kalman_filter(y, x_est, P, A, H, Q, R);
        
        % PID Controller
        current_ref.px = ref.px(i);
        current_ref.py = ref.py(i);
        current_ref.pz = ref.pz(i);
        current_ref.yaw = ref.yaw(i);
        u = pid_controller(current_ref, x_est, params, gains, dt);
        
        % Apply motor fault (only for case 2)
        if case_idx == 2
            [u, fault_active, last_fault_time] = simulate_motor_fault(...
                u, t(i), fault_active, last_fault_time);
        end
        motor_thrusts(i,:) = u'; % Record thrusts
        
        % Update true state
        [~, x_temp] = ode45(@(t,x) quadrotor_model(t, x, u, params), [0, dt], x_true);
        x_true = x_temp(end,:)';
        x_history(i,:) = x_true';
        
        % Track errors
        error.position = [ref.px(i) - x_true(1); ref.py(i) - x_true(2); ref.pz(i) - x_true(3)];
        error.attitude = [0 - x_true(7); 0 - x_true(8); ref.yaw(i) - x_true(9)];
        error_history(i,:) = [error.position; error.attitude];
    end
    
    % Store results
    results(case_idx).x_history = x_history;
    results(case_idx).error_history = error_history;
    results(case_idx).motor_thrusts = motor_thrusts;
end

% ===== Plot Trajectories in Separate Window =====
figure('Name', 'Trajectory Comparison', 'NumberTitle', 'off');
plot3(ref.px, ref.py, ref.pz, 'k-', 'LineWidth', 2); hold on;
plot3(results(1).x_history(:,1), results(1).x_history(:,2), results(1).x_history(:,3), 'b-');
plot3(results(2).x_history(:,1), results(2).x_history(:,2), results(2).x_history(:,3), 'r--');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Quadrotor Trajectory Comparison');
legend('Reference', 'Fault-Free', 'Faulty Motor');
grid on; axis equal;
view(45, 30);

% ===== Plot Errors and Motor Thrusts in Second Window =====
figure('Name', 'Performance Metrics', 'NumberTitle', 'off');

% Position Errors
subplot(2,1,1);
plot(t, results(1).error_history(:,1:3), 'b-'); hold on;
plot(t, results(2).error_history(:,1:3), 'r--');
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Tracking Errors');
legend('X_{fault-free}', 'Y_{fault-free}', 'Z_{fault-free}', ...
       'X_{faulty}', 'Y_{faulty}', 'Z_{faulty}');
grid on;

% Motor Thrusts (Faulty Case Only)
subplot(2,1,2);
plot(t, results(2).motor_thrusts, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Motor Thrusts (Faulty Case)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
grid on;