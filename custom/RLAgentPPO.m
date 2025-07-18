classdef RLAgentPPO
    properties
        % Neural Networks
        actor_net
        critic_net
        actor_optimizer
        critic_optimizer
        
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
        current_episode = 1
        
        % Tracking
        total_reward = 0
        step_count = 0
    end
    
    methods
        function obj = RLAgentPPO(state_dim, action_dim)
            % Initialize actor and critic networks
            obj.actor_net = [
                featureInputLayer(state_dim, 'Name', 'state_input')
                fullyConnectedLayer(64, 'Name', 'fc1')
                reluLayer('Name', 'relu1')
                fullyConnectedLayer(64, 'Name', 'fc2')
                reluLayer('Name', 'relu2')
                fullyConnectedLayer(action_dim, 'Name', 'output')
                tanhLayer('Name', 'tanh_out') % Output between [-1,1]
            ];
            
            obj.critic_net = [
                featureInputLayer(state_dim, 'Name', 'state_input')
                fullyConnectedLayer(64, 'Name', 'fc1')
                reluLayer('Name', 'relu1')
                fullyConnectedLayer(64, 'Name', 'fc2')
                reluLayer('Name', 'relu2')
                fullyConnectedLayer(1, 'Name', 'output')
            ];
            
            % Initialize optimizers
            obj.actor_optimizer = adamOptimizer(obj.learning_rate);
            obj.critic_optimizer = adamOptimizer(obj.learning_rate);
            
            % Initialize memory buffer
            obj.memory_buffer = struct(...
                'states', {}, ...
                'actions', {}, ...
                'rewards', {}, ...
                'next_states', {}, ...
                'dones', {}, ...
                'log_probs', {}, ...
                'values', {}, ...
                'priority', {});
        end
        
        function [action, log_prob] = get_action(obj, state)
            % Forward pass through actor network
            action = predict(obj.actor_net, dlarray(state, 'CB'));
            action = extractdata(action);
            
            % Add exploration noise
            log_prob = log(normpdf(action, 0, 0.1)); % Simple Gaussian noise
        end
        
        function value = get_value(obj, state)
            value = predict(obj.critic_net, dlarray(state, 'CB'));
            value = extractdata(value);
        end
        
        function obj = store_experience(obj, state, action, reward, next_state, done, priority)
            % Store experience in memory buffer
            experience = struct(...
                'state', state, ...
                'action', action, ...
                'reward', reward, ...
                'next_state', next_state, ...
                'done', done, ...
                'log_prob', log(normpdf(action, 0, 0.1)), ...
                'value', obj.get_value(state), ...
                'priority', priority);
            
            if length(obj.memory_buffer) < obj.buffer_capacity
                obj.memory_buffer = [obj.memory_buffer, experience];
            else
                % Replace lowest priority experience
                [~, idx] = min([obj.memory_buffer.priority]);
                obj.memory_buffer(idx) = experience;
            end
        end
        
        function [obj, pid_gains] = update(obj)
            if length(obj.memory_buffer) < obj.buffer_capacity
                pid_gains = [1.5, 1.5, 2.0, 8.0, 8.0, 5.0]; % Default gains
                return;
            end
            
            % Sample batch from memory (prioritized)
            priorities = [obj.memory_buffer.priority];
            sampling_probs = priorities / sum(priorities);
            batch_idx = randsample(1:length(obj.memory_buffer), obj.batch_size, true, sampling_probs);
            batch = obj.memory_buffer(batch_idx);
            
            % PPO Update
            states = cat(2, batch.state);
            actions = cat(2, batch.action);
            old_log_probs = cat(2, batch.log_prob);
            old_values = cat(2, batch.value);
            rewards = cat(2, batch.reward);
            next_states = cat(2, batch.next_state);
            dones = cat(2, batch.done);
            
            % Calculate advantages
            values = obj.get_value(states);
            next_values = obj.get_value(next_states);
            deltas = rewards + obj.gamma * next_values .* ~dones - values;
            advantages = zeros(size(deltas));
            advantage = 0;
            for t = length(deltas):-1:1
                advantage = deltas(t) + obj.gamma * obj.lambda * advantage * ~dones(t);
                advantages(t) = advantage;
            end
            
            % Normalize advantages
            advantages = (advantages - mean(advantages)) / (std(advantages) + 1e-8);
            
            % PPO Loss
            [actor_loss, critic_loss] = ppo_loss(obj, states, actions, old_log_probs, old_values, advantages);
            
            % Update networks
            obj.actor_net = update(obj.actor_optimizer, obj.actor_net, actor_loss);
            obj.critic_net = update(obj.critic_optimizer, obj.critic_net, critic_loss);
            
            % Get new PID gains from actor
            current_state = states(:,1); % Use first state in batch
            pid_gains = extractdata(predict(obj.actor_net, dlarray(current_state, 'CB')));
            
            % Scale gains to reasonable ranges
            pid_gains = [...
                1.0 + 0.5 * pid_gains(1), ... % Kp_pos
                1.0 + 0.5 * pid_gains(2), ...
                2.0 + 1.0 * pid_gains(3), ...
                8.0 + 2.0 * pid_gains(4), ... % Kp_att
                8.0 + 2.0 * pid_gains(5), ...
                5.0 + 2.0 * pid_gains(6)];
        end
        
        function [actor_loss, critic_loss] = ppo_loss(obj, states, actions, old_log_probs, old_values, advantages)
            % PPO Clipped Loss
            new_log_probs = log(normpdf(actions, 0, 0.1)); % Simplified
            ratios = exp(new_log_probs - old_log_probs);
            surr1 = ratios .* advantages;
            surr2 = clip(ratios, 1-obj.clip_epsilon, 1+obj.clip_epsilon) .* advantages;
            actor_loss = -mean(min(surr1, surr2)) + obj.entropy_coeff * mean(new_log_probs);
            
            % Value Loss
            new_values = obj.get_value(states);
            critic_loss = mean(0.5 * (new_values - (old_values + advantages)).^2);
        end
        
        function reward = calculate_reward(obj, error, is_goal_reached, is_moving)
            % Reward components
            error_norm = norm(error);
            threshold = 0.05; % 5% threshold
            
            if error_norm < threshold
                reward = 200; % Large positive reward
            else
                reward = -10 * error_norm; % Proportional punishment
            end
            
            % Living penalty
            if ~is_moving
                reward = reward - 50;
            end
            
            % Goal reward
            if is_goal_reached
                reward = reward + 2000;
            end
            
            % Clip total reward
            reward = min(reward, 20000);
        end
    end
end