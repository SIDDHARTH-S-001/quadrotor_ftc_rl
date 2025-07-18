%% Main Script with RL-PPO Agent (Enhanced Tracking)
clear; clc; close all;

% ===== Initialize Systems =====
[params, gains, ref, t] = initialize_systems();

% ===== RL Agent =====
agent = RLAgentPPO(12, 6); % state_dim=12, action_dim=6

% ===== Simulation Loop =====
max_episodes = 10;
results = struct();
training_log = struct('buffer_size', [], 'total_reward', [], 'loss', []);

% Create figure for live training progress
progress_fig = figure('Name', 'Training Progress', 'Position', [100 100 1200 600]);

for episode = 1:max_episodes
    % Initialize state
    [x_true, x_est, P] = initialize_state(ref);
    
    % Episode tracking variables
    x_history = zeros(length(t), 12);
    error_history = zeros(length(t), 6);
    rewards = zeros(length(t), 1);
    prev_position = x_true(1:3);
    
    for i = 1:length(t)
        % Get current reference
        current_ref = get_current_reference(ref, i);
        
        % Kalman Filter
        [x_est, P] = update_kalman_filter(x_true, x_est, P, params);
        
        % RL Agent action (PID gains)
        [pid_gains, ~] = agent.get_action(x_est);
        gains = update_pid_gains(gains, pid_gains);
        
        % PID Controller
        u = pid_controller(current_ref, x_est, params, gains, params.dt);
        
        % Update true state
        x_true = update_drone_state(x_true, u, params);
        x_history(i,:) = x_true';
        
        % Calculate error and reward
        [error, is_moving, is_goal_reached] = calculate_performance(x_true, current_ref, prev_position, i, length(t));
        reward = agent.calculate_reward(error, is_goal_reached, is_moving);
        
        % Store experience
        priority = norm(error);
        agent = agent.store_experience(x_est, pid_gains, reward, x_true, is_goal_reached, priority);
        
        % Update agent and get training metrics if buffer is full
        [agent, pid_gains, train_metrics] = agent.update();
        
        % Store results and update training log
        [results, training_log] = store_results(results, training_log, episode, i, ...
            x_history, error_history, rewards, reward, error, agent, train_metrics);
        
        prev_position = x_true(1:3);
    end
    
    % Display episode summary
    display_episode_summary(episode, results, agent, training_log);
    
    % Update progress plot
    update_progress_plot(progress_fig, training_log, episode, max_episodes);
end

% Final plots
plot_final_results(results, ref, t);

%% Helper Functions
function [params, gains, ref, t] = initialize_systems()
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

    t = 0:params.dt:10;
    
    ref.px = sin(pi*t/500);
    ref.py = sin(pi*t/500);
    ref.pz = ones(size(t));
    ref.vx = (pi/500)*cos(pi*t/500);
    ref.vy = (pi/500)*cos(pi*t/500);
    ref.vz = zeros(size(t));
    ref.yaw = zeros(size(t));

    gains.Kp_pos = [1.5, 1.5, 2.0];
    gains.Ki_pos = [0.1, 0.1, 0.2];
    gains.Kd_pos = [0.5, 0.5, 0.8];
    gains.Kp_att = [8.0, 8.0, 5.0];
    gains.Ki_att = [0.5, 0.5, 0.3];
    gains.Kd_att = [2.0, 2.0, 1.5];
end

function [x_true, x_est, P] = initialize_state(ref)
    x_true = [ref.px(1); ref.py(1); ref.pz(1); ...
              ref.vx(1); ref.vy(1); ref.vz(1); ...
              0; 0; ref.yaw(1); 0; 0; 0];
    x_est = x_true;
    P = eye(12);
end

function current_ref = get_current_reference(ref, i)
    current_ref.px = ref.px(i);
    current_ref.py = ref.py(i);
    current_ref.pz = ref.pz(i);
    current_ref.yaw = ref.yaw(i);
end

function [x_est, P] = update_kalman_filter(x_true, x_est, P, params)
    y = x_true + sqrt(params.R)*randn(12,1);
    [x_est, P] = kalman_filter(y, x_est, P, params.A, params.H, params.Q, params.R);
end

function gains = update_pid_gains(gains, pid_gains)
    gains.Kp_pos = pid_gains(1:3);
    gains.Kp_att = pid_gains(4:6);
end

function x_true = update_drone_state(x_true, u, params)
    [~, x_temp] = ode45(@(t,x) quadrotor_model(t, x, u, params), [0, params.dt], x_true);
    x_true = x_temp(end,:)';
end

function [error, is_moving, is_goal_reached] = calculate_performance(x_true, current_ref, prev_position, i, t_len)
    error = [current_ref.px - x_true(1);
             current_ref.py - x_true(2);
             current_ref.pz - x_true(3)];
    is_moving = norm(x_true(1:3) - prev_position) > 0.01;
    is_goal_reached = (i == t_len) && (norm(error) < 0.05);
end

function [results, training_log] = store_results(results, training_log, episode, i, ...
        x_history, error_history, rewards, reward, error, agent, train_metrics)
    
    error_history(i,:) = [error; 0; 0; 0];
    rewards(i) = reward;
    
    results(episode).x_history = x_history;
    results(episode).error_history = error_history;
    results(episode).rewards = rewards;
    
    % Store training metrics if buffer was full
    if ~isempty(train_metrics)
        training_log.buffer_size(end+1) = length(agent.memory_buffer);
        training_log.total_reward(end+1) = sum(rewards(1:i));
        training_log.loss(end+1) = train_metrics.loss;
    end
end

function display_episode_summary(episode, results, agent, training_log)
    fprintf('\n=== Episode %d Summary ===\n', episode);
    fprintf('Total Reward: %.2f\n', sum(results(episode).rewards));
    fprintf('Memory Buffer Size: %d/%d\n', length(agent.memory_buffer), agent.buffer_capacity);
    
    if length(agent.memory_buffer) >= agent.buffer_capacity
        fprintf('Training Metrics:\n');
        fprintf('  - Average Loss: %.4f\n', mean(training_log.loss(end-9:end)));
    end
end

function update_progress_plot(fig, training_log, episode, max_episodes)
    figure(fig);
    clf;
    
    if isfield(training_log, 'buffer_size') && ~isempty(training_log.buffer_size)
        % Buffer Size Plot
        subplot(2,2,1);
        plot(training_log.buffer_size, 'LineWidth', 2);
        xlabel('Training Step');
        ylabel('Buffer Size');
        title('Memory Buffer Usage');
        grid on;
        
        % Reward Plot
        subplot(2,2,2);
        plot(training_log.total_reward, 'LineWidth', 2);
        xlabel('Training Step');
        ylabel('Total Reward');
        title('Training Reward Progress');
        grid on;
        
        % Loss Plot
        subplot(2,2,3);
        plot(training_log.loss, 'LineWidth', 2);
        xlabel('Training Step');
        ylabel('Loss');
        title('Training Loss');
        grid on;
        
        % Episode Progress
        subplot(2,2,4);
        bar([episode/max_episodes, length(training_log.buffer_size)/500]);
        set(gca, 'XTickLabel', {'Episode Progress', 'Buffer Fill %'});
        ylim([0 1]);
        title('Training Progress');
    else
        text(0.5, 0.5, sprintf('Filling Memory Buffer...\n(%d/%d experiences collected)', ...
            length(training_log.buffer_size), 500), ...
            'HorizontalAlignment', 'center', 'FontSize', 14);
    end
    drawnow;
end

function plot_final_results(results, ref, t)
    figure('Name', 'Final Results', 'Position', [100 100 1200 400]);
    
    % Trajectory Comparison
    subplot(1,3,1);
    plot3(ref.px, ref.py, ref.pz, 'k-', 'LineWidth', 2); hold on;
    plot3(results(end).x_history(:,1), results(end).x_history(:,2), results(end).x_history(:,3), 'b-');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Final Trajectory');
    legend('Reference', 'Actual');
    grid on; axis equal;
    view(45, 30);
    
    % Learning Curve
    subplot(1,3,2);
    plot(arrayfun(@(x) sum(x.rewards), results), 'LineWidth', 2);
    xlabel('Episode'); ylabel('Total Reward');
    title('Learning Curve');
    grid on;
    
    % Final Episode Errors
    subplot(1,3,3);
    plot(t, results(end).error_history(:,1:3));
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Final Episode Errors');
    legend('X', 'Y', 'Z');
    grid on;
end