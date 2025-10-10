function [x_est, P_est] = kalman_filter(y, x_prev, P_prev, A, H, Q, R)
    % 1) Prediction
    x_pred = A*x_prev;
    P_pred = A*P_prev*A' + Q;

    % 2) Update
    K = P_pred*H'/(H*P_pred*H' + R);
    x_est  = x_pred + K*(y - H*x_pred);
    P_est  = (eye(size(P_pred)) - K*H)*P_pred;

    % enforce symmetry
    P_est = (P_est + P_est')/2;
end
