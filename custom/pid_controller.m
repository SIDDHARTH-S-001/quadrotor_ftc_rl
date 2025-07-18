function [u] = pid_controller(ref, x_est, params, gains, dt)
    % PID Controller for Quadrotor
    % Inputs:
    %   ref: reference trajectory (struct with px, py, pz, yaw)
    %   x_est: estimated state from Kalman filter [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    %   params: system parameters (mass, gravity, etc.)
    %   gains: struct with PID gains (Kp, Ki, Kd for position and attitude)
    %   dt: time step
    % Output:
    %   u: control inputs [tau1, tau2, tau3, tau4]

    % Initialize persistent variables for integral and derivative terms
    persistent integral_error prev_error;
    if isempty(integral_error)
        integral_error = zeros(6, 1); % [x, y, z, phi, theta, psi]
        prev_error = zeros(6, 1);
    end

    % Extract estimated states
    px_est = x_est(1); py_est = x_est(2); pz_est = x_est(3);
    phi_est = x_est(7); theta_est = x_est(8); psi_est = x_est(9);

    % Calculate errors
    error = [ref.px - px_est;
             ref.py - py_est;
             ref.pz - pz_est;
             0 - phi_est;      % Reference roll is 0
             0 - theta_est;    % Reference pitch is 0
             ref.yaw - psi_est]; % Reference yaw

    % Update integral and derivative terms
    integral_error = integral_error + error * dt;
    derivative_error = (error - prev_error) / dt;
    prev_error = error;

    % PID Control (position and attitude)
    % Position control (outer loop)
    F_des = gains.Kp_pos(1:3) .* error(1:3) + ...
             gains.Ki_pos(1:3) .* integral_error(1:3) + ...
             gains.Kd_pos(1:3) .* derivative_error(1:3);

    % Attitude control (inner loop)
    tau_phi = gains.Kp_att(1) * error(4) + gains.Ki_att(1) * integral_error(4) + gains.Kd_att(1) * derivative_error(4);
    tau_theta = gains.Kp_att(2) * error(5) + gains.Ki_att(2) * integral_error(5) + gains.Kd_att(2) * derivative_error(5);
    tau_psi = gains.Kp_att(3) * error(6) + gains.Ki_att(3) * integral_error(6) + gains.Kd_att(3) * derivative_error(6);

    % Total thrust (tau4) is mg + F_des_z (compensate gravity)
    tau4 = params.m * params.g + F_des(3);

    % Convert desired forces to roll/pitch commands (simplified)
    phi_des = (F_des(1) * sin(psi_est) - F_des(2) * cos(psi_est)) / tau4;
    theta_des = (F_des(1) * cos(psi_est) + F_des(2) * sin(psi_est)) / tau4;

    % Attitude control inputs (tau1, tau2, tau3)
    tau1 = (phi_des - phi_est) * params.Ix;
    tau2 = (theta_des - theta_est) * params.Iy;
    tau3 = tau_psi * params.Iz;

    % Saturation (optional, to limit control inputs)
    u = [tau1; tau2; tau3; tau4];
end