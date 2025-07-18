%% Main Script for Quadrotor Control with PID and Kalman Filter
clear; clc; close all;

% ===== Define Parameters =====
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
gains.Kp_pos = [1.5, 1.5, 2.0];  % Position [x, y, z]
gains.Ki_pos = [0.1, 0.1, 0.2];   % Integral gains
gains.Kd_pos = [0.5, 0.5, 0.8];   % Derivative gains
gains.Kp_att = [8.0, 8.0, 5.0];   % Attitude [phi, theta, psi]
gains.Ki_att = [0.5, 0.5, 0.3];   % Integral gains
gains.Kd_att = [2.0, 2.0, 1.5];   % Derivative gains

% ===== Kalman Filter Setup =====
dt = 0.01; % Time step
A = eye(12); % State transition matrix (simplified)
H = eye(12); % Measurement matrix (assume all states are measured)
Q = diag(0.01 * ones(12,1)); % Process noise covariance
R = diag(0.1 * ones(12,1));  % Measurement noise covariance

% ===== Time Vector =====
t = 0:dt:10; % 0 to 10s

% ===== Generate Reference Trajectory =====
ref = generate_reference(t);

% ===== Initialize State and Covariance =====
x_true = [ref.px(1); ref.py(1); ref.pz(1); ... % Initial position
          ref.vx(1); ref.vy(1); ref.vz(1); ... % Initial velocity
          0; 0; ref.yaw(1); ...                % Initial attitude
          0; 0; 0];                            % Initial angular rates
x_est = x_true; % Initial estimate
P = eye(12);    % Initial covariance

% ===== Simulation Loop =====
x_history = zeros(length(t), 12);
error_history = zeros(length(t), 6);

for i = 1:length(t)
    % Simulate sensor measurements (add noise)
    y = x_true + sqrt(R) * randn(12,1);

    % Kalman Filter
    [x_est, P] = kalman_filter(y, x_est, P, A, H, Q, R);

    % Calculate error
    error = calculate_error(ref, x_est);
    error_history(i,:) = [error.position; error.attitude];

    % PID Controller
    u = pid_controller(ref, x_est, params, gains, dt);

    % Update true state (using quadrotor_model)
    [~, x_temp] = ode45(@(t,x) quadrotor_model(t, x, u, params), [0, dt], x_true);
    x_true = x_temp(end,:)';

    % Store history
    x_history(i,:) = x_true';
end

% ===== Plot Results =====
figure;
subplot(2,1,1);
plot3(ref.px, ref.py, ref.pz, 'b-', 'LineWidth', 2); hold on;
plot3(x_history(:,1), x_history(:,2), x_history(:,3), 'r--', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Quadrotor Trajectory Tracking');
legend('Reference', 'Actual');
grid on; axis equal;
view(45, 30);

subplot(2,1,2);
plot(t, error_history(:,1:3), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Tracking Error');
legend('X-error', 'Y-error', 'Z-error');
grid on;