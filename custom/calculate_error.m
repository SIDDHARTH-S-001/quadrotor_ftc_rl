function [error] = calculate_error(ref, x_est, i)
    % Calculate Tracking Error Between Reference and Estimated State
    % Inputs:
    %   ref: reference trajectory (struct with arrays px, py, pz, yaw)
    %   x_est: estimated state vector [12x1]
    %   i: current time index
    % Output:
    %   error: struct with position and attitude errors (scalars)

    error.position = [ref.px(i) - x_est(1);  % X-error
                     ref.py(i) - x_est(2);   % Y-error
                     ref.pz(i) - x_est(3)];  % Z-error
    error.attitude = [0 - x_est(7);          % Roll error (reference = 0)
                     0 - x_est(8);           % Pitch error (reference = 0)
                     ref.yaw(i) - x_est(9)]; % Yaw error
end