function [u_faulty, fault_active, last_fault_time] = simulate_motor_fault(u, t, fault_active, last_fault_time)
    % Simulate motor fault (30% chance, 50% thrust loss for 0.5s, 1s cooldown)
    % Inputs:
    %   u: Original control inputs [tau1, tau2, tau3, tau4]
    %   t: Current time
    %   fault_active: Boolean indicating if a fault is currently active
    %   last_fault_time: Time when the last fault ended
    % Outputs:
    %   u_faulty: Modified control inputs with fault
    %   fault_active: Updated fault status
    %   last_fault_time: Updated last fault time

    persistent fault_motor;
    
    % Initialize persistent variables
    if isempty(fault_motor)
        fault_motor = 0; % 1-4 for motor index, 0 for no fault
    end

    % Check if fault duration has expired (0.5s)
    if fault_active && (t - last_fault_time >= 0.5)
        fault_active = false;
        last_fault_time = t;
    end

    % Random fault trigger (10% probability during cooldown)
    if ~fault_active && (t - last_fault_time >= 1.0) && (rand() <= 0.30)
        fault_active = true;
        fault_motor = randi(4); % Random motor (1-4)
        last_fault_time = t;
    end

    % Apply fault if active
    u_faulty = u;
    if fault_active
        u_faulty(fault_motor) = u(fault_motor) * 0.5; % 50% thrust loss
    end
end