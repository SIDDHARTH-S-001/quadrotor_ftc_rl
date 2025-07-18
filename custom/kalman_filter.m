function [x_est, P] = kalman_filter(y, x_pred, P_pred, A, H, Q, R)
    % Linear Kalman Filter for Quadrotor State Estimation
    % Inputs:
    %   y: measurement vector (sensor data)
    %   x_pred: predicted state from previous step
    %   P_pred: predicted covariance from previous step
    %   A: state transition matrix
    %   H: measurement matrix
    %   Q: process noise covariance
    %   R: measurement noise covariance
    % Outputs:
    %   x_est: posterior state estimate
    %   P: posterior covariance estimate

    % Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R);

    % Update estimate with measurement
    x_est = x_pred + K * (y - H * x_pred);

    % Update covariance
    P = (eye(size(P_pred)) - K * H) * P_pred;

    % Optional: Ensure P remains symmetric
    P = (P + P') / 2;
end