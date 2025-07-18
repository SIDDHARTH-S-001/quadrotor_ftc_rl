function [ref] = generate_reference(t)
    % Sinusoidal reference trajectory (from paper: P_ref = [sin(πt/500), sin(πt/500), 1])
    % Input: 
    %   t: time vector
    % Output:
    %   ref: struct with reference positions, velocities, and yaw angle

    ref.px = sin(pi * t / 500); % X-position
    ref.py = sin(pi * t / 500); % Y-position
    ref.pz = ones(size(t));     % Z-position (constant height)
    ref.vx = (pi/500) * cos(pi * t / 500); % X-velocity
    ref.vy = (pi/500) * cos(pi * t / 500); % Y-velocity
    ref.vz = zeros(size(t));    % Z-velocity
    ref.yaw = zeros(size(t));   % Yaw angle (0° as in paper)
end