%% Main Script with RL-PPO Agent
clear; clc; close all;

% ===== Initialize Systems =====
params = struct();
params.m = 1.0;
params.g = 9.81;
params.Ix = 0.01;
params.Iy = 0.01;
params.Iz = 0.02;
params.L = 0.2;
params.JR = 1e-4;
params.OmegaR = 0;
params.d = zeros(6,1);
params.dt = 0.01;
params.A = eye(12);
params.H = eye(12);
params.Q = diag(0.01*ones(12,1));
params.R = diag(0.1*ones(12,1));

% Time vector
t = 0:params.dt:10;

% Reference trajectory
ref.px = sin(pi*t/500);
ref.py = sin(pi*t/500);
ref.pz = ones(size(t));
ref.vx = (pi/500)*cos(pi*t/500);
ref.vy = (pi/500)*cos(pi*t/500);
ref.vz = zeros(size(t));
ref.yaw = zeros(size(t));

% Default PID gains
gains.Kp_pos = [1.5, 1.5, 2.0];
gains.Ki_pos = [0.1, 0.1, 0.2];
gains.Kd_pos = [0.5, 0.5, 0.8];
gains.Kp_att = [8.0, 8.0, 5.0];
gains.Ki_att = [0.5, 0.5, 0.3];
gains.Kd_att = [2.0, 2.0, 1.5];

% ===== RL Agent =====
agent = RLAgentPPO(12, 6); % state_dim=12, action_dim=6

% ===== Simulation Loop =====
max_episodes = 10;
results = struct();

for episode = 1:max_episodes
    % Initialize state (ensure column vector)
    x_true = [ref.px(1); ref.py(1); ref.pz(1); ...
              ref.vx(1); ref.vy(1); ref.vz(1); ...
              0; 0; ref.yaw(1); 0; 0; 0];
    x_est = x_true;
    P = eye(12);
    
    x_history = zeros(length(t), 12);
    error_history = zeros(length(t), 6);
    rewards = zeros(length(t), 1);
    prev_position = x_true(1:3);
    
    for i = 1:length(t)
        % Get current reference
        current_ref.px = ref.px(i);
        current_ref.py = ref.py(i);
        current_ref.pz = ref.pz(i);
        current_ref.yaw = ref.yaw(i);
        
        % Kalman Filter
        y = x_true + sqrt(params.R)*randn(12,1);
        [x_est, P] = kalman_filter(y, x_est, P, params.A, params.H, params.Q, params.R);
        
        % RL Agent action (PID gains)
        [pid_gains, ~] = agent.get_action(x_est);
        gains.Kp_pos = pid_gains(1:3);
        gains.Kp_att = pid_gains(4:6);
        
        % PID Controller
        u = pid_controller(current_ref, x_est, params, gains, params.dt);
        
        % Update true state
        [~, x_temp] = ode45(@(t,x) quadrotor_model(t, x, u, params), [0, params.dt], x_true);
        x_true = x_temp(end,:)';
        x_history(i,:) = x_true';
        
        % Calculate error and reward
        error = [current_ref.px - x_true(1);
                 current_ref.py - x_true(2);
                 current_ref.pz - x_true(3)];
        is_moving = norm(x_true(1:3) - prev_position) > 0.01;
        is_goal_reached = (i == length(t)) && (norm(error) < 0.05);
        reward = agent.calculate_reward(error, is_goal_reached, is_moving);
        
        % Store experience
        priority = norm(error);
        agent = agent.store_experience(x_est, pid_gains, reward, x_true, is_goal_reached, priority);
        
        % Update agent
        [agent, ~] = agent.update();
        
        % Store results
        error_history(i,:) = [error; 0; 0; 0];
        rewards(i) = reward;
        prev_position = x_true(1:3);
    end
    
    results(episode).x_history = x_history;
    results(episode).error_history = error_history;
    results(episode).rewards = rewards;
    fprintf('Episode %d: Total Reward = %.2f\n', episode, sum(rewards));
end

% ===== Plot Results =====
figure('Name', 'Trajectory Comparison');
plot3(ref.px, ref.py, ref.pz, 'k-', 'LineWidth', 2); hold on;
plot3(results(end).x_history(:,1), results(end).x_history(:,2), results(end).x_history(:,3), 'b-');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Final Episode Trajectory');
legend('Reference', 'Actual');
grid on; axis equal;
view(45, 30);

figure('Name', 'Learning Performance');
plot(arrayfun(@(x) sum(x.rewards), results), 'LineWidth', 2);
xlabel('Episode'); ylabel('Total Reward');
title('Learning Curve');
grid on;