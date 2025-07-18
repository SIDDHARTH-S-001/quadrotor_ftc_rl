classdef RLAgentPPO
    properties
        % Neural Networks
        actor_net
        critic_net
        
        % Hyperparameters
        clip_epsilon = 0.2
        gamma = 0.99
        lambda = 0.95
        learning_rate = 3e-4
        entropy_coeff = 0.01
        
        % Memory Buffer
        memory_buffer
        buffer_capacity = 500
        batch_size = 64
        
        % Tracking
        total_reward = 0
        step_count = 0
    end
    
    methods
        function obj = RLAgentPPO(state_dim, action_dim)
            % Initialize actor network
            actor_layers = [
                featureInputLayer(state_dim, 'Name', 'input')
                fullyConnectedLayer(64, 'Name', 'fc1')
                reluLayer('Name', 'relu1')
                fullyConnectedLayer(64, 'Name', 'fc2')
                reluLayer('Name', 'relu2')
                fullyConnectedLayer(action_dim, 'Name', 'output')
                tanhLayer('Name', 'tanh_out')
            ];
            obj.actor_net = dlnetwork(actor_layers);
            
            % Initialize critic network
            critic_layers = [
                featureInputLayer(state_dim, 'Name', 'input')
                fullyConnectedLayer(64, 'Name', 'fc1')
                reluLayer('Name', 'relu1')
                fullyConnectedLayer(64, 'Name', 'fc2')
                reluLayer('Name', 'relu2')
                fullyConnectedLayer(1, 'Name', 'output')
            ];
            obj.critic_net = dlnetwork(critic_layers);
            
            % Initialize memory buffer with empty struct of correct fields
            obj.memory_buffer = struct(...
                'state', {}, ...
                'action', {}, ...
                'reward', {}, ...
                'next_state', {}, ...
                'done', {}, ...
                'log_prob', {}, ...
                'value', {}, ...
                'priority', {});
        end
        
        function [action, log_prob] = get_action(obj, state)
            % Convert state to correctly formatted dlarray
            state_dl = dlarray(single(state(:)), 'CB');
            
            % Forward pass through actor network
            action = predict(obj.actor_net, state_dl);
            action = extractdata(action)';
            
            % Add exploration noise
            noise = 0.1 * randn(size(action));
            action = action + noise;
            log_prob = sum(log(normpdf(noise, 0, 0.1)));
        end
        
        function value = get_value(obj, state)
            state_dl = dlarray(single(state(:)), 'CB');
            value = predict(obj.critic_net, state_dl);
            value = extractdata(value);
        end
        
        function obj = store_experience(obj, state, action, reward, next_state, done, priority)
            % Create experience with consistent field names
            experience = struct(...
                'state', {state}, ...
                'action', {action}, ...
                'reward', {reward}, ...
                'next_state', {next_state}, ...
                'done', {done}, ...
                'log_prob', {log(normpdf(action - mean(action), 0, 0.1))}, ...
                'value', {obj.get_value(state)}, ...
                'priority', {priority});
            
            if length(obj.memory_buffer) < obj.buffer_capacity
                obj.memory_buffer = [obj.memory_buffer; experience];
            else
                [~, idx] = min([obj.memory_buffer.priority]);
                obj.memory_buffer(idx) = experience;
            end
        end
        
        function [obj, pid_gains] = update(obj)
            if length(obj.memory_buffer) < obj.buffer_capacity
                pid_gains = [1.5, 1.5, 2.0, 8.0, 8.0, 5.0]; % Default gains
                return;
            end
            
            % Sample batch
            batch_idx = randperm(length(obj.memory_buffer), min(obj.batch_size, length(obj.memory_buffer)));
            batch = obj.memory_buffer(batch_idx);
            
            % Prepare data with correct dimensions
            states = [batch.state];
            states_dl = dlarray(single(states), 'CB');
            
            actions = [batch.action];
            old_values = [batch.value];
            rewards = [batch.reward];
            next_states = [batch.next_state];
            dones = [batch.done];
            
            % Calculate advantages
            next_values = arrayfun(@(s) obj.get_value(s), num2cell(next_states,1));
            deltas = rewards + obj.gamma * next_values .* ~dones - old_values;
            
            advantages = zeros(size(deltas));
            advantage = 0;
            for t = length(deltas):-1:1
                advantage = deltas(t) + obj.gamma * obj.lambda * advantage * ~dones(t);
                advantages(t) = advantage;
            end
            advantages = (advantages - mean(advantages)) / (std(advantages) + 1e-8);
            
            % Get current PID gains
            current_state = states(:,1);
            [pid_gains, ~] = obj.get_action(current_state);
            pid_gains = [
                1.0 + 0.5 * pid_gains(1), ... % Kp_pos
                1.0 + 0.5 * pid_gains(2), ...
                2.0 + 1.0 * pid_gains(3), ...
                8.0 + 2.0 * pid_gains(4), ... % Kp_att
                8.0 + 2.0 * pid_gains(5), ...
                5.0 + 2.0 * pid_gains(6)];
        end
        
        function reward = calculate_reward(obj, error, is_goal_reached, is_moving)
            error_norm = norm(error);
            threshold = 0.05;
            
            if error_norm < threshold
                reward = 200;
            else
                reward = -10 * error_norm;
            end
            
            if ~is_moving
                reward = reward - 50;
            end
            
            if is_goal_reached
                reward = reward + 2000;
            end
            
            reward = min(reward, 20000);
        end
    end
end