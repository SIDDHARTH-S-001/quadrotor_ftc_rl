%% Main Script for Quadrotor Trajectory Simulation
clear; clc; close all;

% ===== Define Parameters =====
params.m = 1.0;     % Mass (kg)
params.g = 9.81;    % Gravity (m/s²)
params.Ix = 0.01;   % Moment of inertia (X-axis)
params.Iy = 0.01;   % Moment of inertia (Y-axis)
params.Iz = 0.02;   % Moment of inertia (Z-axis)
params.L = 0.2;     % Arm length (m)
params.JR = 1e-4;   % Propeller inertia (placeholder)
params.OmegaR = 0;  % Angular velocity of propellers (assumed zero for simplicity)
params.d = zeros(6,1); % Disturbances (set to zero for now)

% ===== Time Vector =====
t = linspace(0, 100, 1000); % 0 to 100s, 1000 points

% ===== Generate Reference Trajectory =====
ref = generate_reference(t);

% ===== Plot Reference Trajectory =====
figure;
plot3(ref.px, ref.py, ref.pz, 'b-', 'LineWidth', 2); 
hold on;
scatter3(ref.px(1), ref.py(1), ref.pz(1), 100, 'g', 'filled'); % Start point
scatter3(ref.px(end), ref.py(end), ref.pz(end), 100, 'r', 'filled'); % End point
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Quadrotor Reference Trajectory (Sinusoidal)');
legend('Trajectory', 'Start', 'End');
grid on; axis equal;
view(45, 30); % Adjust view angle for 3D plot

% ===== Simulate Quadrotor Dynamics (Open-Loop, No Control) =====
% Note: This is a placeholder. Actual control (PID/RL) will be added later.
x0 = [ref.px(1); ref.py(1); ref.pz(1); ... % Initial position
      ref.vx(1); ref.vy(1); ref.vz(1); ... % Initial velocity
      0; 0; ref.yaw(1); ...                % Initial attitude (roll, pitch, yaw)
      0; 0; 0];                            % Initial angular rates
u = [0; 0; 0; params.m * params.g];        % Neutral control input (hover)
[~, x] = ode45(@(t,x) quadrotor_model(t, x, u, params), t, x0);

% Plot simulated trajectory (open-loop)
plot3(x(:,1), x(:,2), x(:,3), 'k--', 'LineWidth', 1.5);
legend('Reference', 'Start', 'End', 'Open-Loop Drone');