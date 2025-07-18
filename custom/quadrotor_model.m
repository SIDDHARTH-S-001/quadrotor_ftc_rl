function [dx] = quadrotor_model(t, x, u, params)
    % Quadrotor dynamics model (Equation 1 from the paper)
    % Inputs:
    %   t: time (unused if dynamics are time-invariant)
    %   x: state vector [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    %   u: control inputs [tau1, tau2, tau3, tau4]
    %   params: struct with system parameters (mass, inertia, etc.)
    % Output:
    %   dx: derivative of the state vector

    % Extract states
    px = x(1); py = x(2); pz = x(3);         % Position (unused in dynamics)
    vx = x(4); vy = x(5); vz = x(6);         % Velocity
    phi = x(7); theta = x(8); psi = x(9);    % Euler angles
    p = x(10); q = x(11); r = x(12);         % Angular rates

    % Extract inputs (thrusts)
    tau1 = u(1); tau2 = u(2); tau3 = u(3); tau4 = u(4);

    % Extract parameters
    m = params.m; g = params.g;
    Ix = params.Ix; Iy = params.Iy; Iz = params.Iz;
    L = params.L; JR = params.JR; OmegaR = params.OmegaR;
    d = params.d; % Disturbances [d1, d2, d3, d4, d5, d6]

    % Translational dynamics (Equation 1, first 3 rows)
    dx(1:3) = [vx; vy; vz]; % Position derivatives
    dvx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * tau4 / m + d(1);
    dvy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * tau4 / m + d(2);
    dvz = (cos(theta)*cos(phi)) * tau4 / m - g + d(3);
    dx(4:6) = [dvx; dvy; dvz];

    % Rotational dynamics (Equation 1, last 3 rows)
    dp = (q*r*(Iy - Iz) - JR*q*OmegaR + L*tau1) / Ix + d(4);
    dq = (p*r*(Iz - Ix) + JR*p*OmegaR + L*tau2) / Iy + d(5);
    dr = (p*q*(Ix - Iy) + tau3) / Iz + d(6);
    dx(7:9) = [p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;  % phi_dot
               cos(phi)*q - sin(phi)*r;                           % theta_dot
               sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r];     % psi_dot
    dx(10:12) = [dp; dq; dr];

    dx = dx'; % Ensure column vector
end