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
        training_log = struct('buffer_size', [], 'total_reward', [], 'loss', [])
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
            
            % Initialize memory buffer
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
            % Ensure state is column vector and convert to dlarray
            state = state(:); % Force column vector
            state_dl = dlarray(single(state'), 'CB'); % Transpose for correct 'CB' format
            
            % Forward pass through actor network
            action = predict(obj.actor_net, state_dl);
            action = extractdata(action)';
            
            % Add exploration noise
            noise = 0.1 * randn(size(action));
            action = action + noise;
            log_prob = sum(log(normpdf(noise, 0, 0.1)));
        end
        
        function value = get_value(obj, state)
            % Ensure state is column vector and convert to dlarray
            state = state(:); % Force column vector
            state_dl = dlarray(single(state'), 'CB'); % Transpose for correct 'CB' format
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
            
            % Update tracking
            obj.step_count = obj.step_count + 1;
            obj.total_reward = obj.total_reward + reward;
        end
        
        function [obj, pid_gains, train_metrics] = update(obj)
            % Initialize empty metrics
            train_metrics = struct('loss', NaN, 'buffer_size', length(obj.memory_buffer));
            
            if length(obj.memory_buffer) < obj.buffer_capacity
                pid_gains = [1.5, 1.5, 2.0, 8.0, 8.0, 5.0]; % Default gains
                fprintf('Buffer filling: %d/%d\n', length(obj.memory_buffer), obj.buffer_capacity);
                return;
            end
            
            % Sample batch
            batch_idx = randperm(length(obj.memory_buffer), min(obj.batch_size, length(obj.memory_buffer)));
            batch = obj.memory_buffer(batch_idx);
            
            % Prepare data with proper dimensions
            states = cat(2, batch.state); % Concatenate states horizontally
            actions = [batch.action];
            old_values = [batch.value];
            rewards = [batch.reward];
            next_states = cat(2, batch.next_state);
            dones = [batch.done];
            
            % Calculate advantages
            next_values = zeros(1, size(next_states,2));
            for i = 1:size(next_states,2)
                next_values(i) = obj.get_value(next_states(:,i));
            end
            
            deltas = rewards + obj.gamma * next_values .* ~dones - old_values;
            
            advantages = zeros(size(deltas));
            advantage = 0;
            for t = length(deltas):-1:1
                advantage = deltas(t) + obj.gamma * obj.lambda * advantage * ~dones(t);
                advantages(t) = advantage;
            end
            advantages = (advantages - mean(advantages)) / (std(advantages) + 1e-8);
            
            % Calculate loss
            states_dl = dlarray(single(states), 'CB'); % Proper batch format
            current_values = predict(obj.critic_net, states_dl);
            value_loss = mean(0.5 * (extractdata(current_values) - (old_values + advantages)').^2);
            
            % Store training metrics
            train_metrics.loss = value_loss;
            train_metrics.buffer_size = length(obj.memory_buffer);
            
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
            
            % Display training progress
            fprintf('Training - Loss: %.4f | Buffer: %d/%d | Avg Reward: %.2f\n', ...
                value_loss, length(obj.memory_buffer), obj.buffer_capacity, mean(rewards));
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
        
        function print_progress(obj, episode)
            fprintf('\n=== Episode %d Summary ===\n', episode);
            fprintf('Total Steps: %d\n', obj.step_count);
            fprintf('Total Reward: %.2f\n', obj.total_reward);
            fprintf('Memory Buffer: %d/%d (%.1f%%)\n', ...
                length(obj.memory_buffer), obj.buffer_capacity, ...
                100*length(obj.memory_buffer)/obj.buffer_capacity);
            
            if length(obj.memory_buffer) >= obj.buffer_capacity
                fprintf('Training Active\n');
                if ~isempty(obj.training_log.loss)
                    fprintf('Recent Loss: %.4f\n', obj.training_log.loss(end));
                end
            else
                fprintf('Collecting Experiences...\n');
            end
        end
    end
end