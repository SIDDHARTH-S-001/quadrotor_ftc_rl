function [error] = calculate_error(ref, x_est)
    % Calculate Tracking Error Between Reference and Estimated State
    % Inputs:
    %   ref: reference trajectory (struct with px, py, pz, yaw)
    %   x_est: estimated state [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    % Output:
    %   error: struct with position and attitude errors

    error.position = [ref.px - x_est(1);  % X-error
                      ref.py - x_est(2);  % Y-error
                      ref.pz - x_est(3)]; % Z-error
    error.attitude = [0 - x_est(7);       % Roll error (reference = 0)
                      0 - x_est(8);       % Pitch error (reference = 0)
                      ref.yaw - x_est(9)]; % Yaw error
end