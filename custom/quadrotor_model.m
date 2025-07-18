function dx = quadrotor_model(t, x, u, params)
    % quadrotor_model 12-state dynamics for a +-quadrotor
    %
    % x = [px;py;pz; vx;vy;vz; phi;theta;psi; p;q;r]
    % u = [tau1;tau2;tau3;tau4] (individual rotor thrusts)
    % params.dist = [d1…d6] disturbance vector

    % Preallocate
    dx = zeros(12,1);

    % Unpack states
    vx    = x(4); vy    = x(5); vz    = x(6);
    phi   = x(7); theta = x(8); psi   = x(9);
    p     = x(10); q    = x(11); r     = x(12);

    % Unpack inputs
    tau1 = u(1); tau2 = u(2);
    tau3 = u(3); tau4 = u(4);
    T    = tau1 + tau2 + tau3 + tau4;

    % Unpack disturbances
    d1 = params.dist(1);
    d2 = params.dist(2);
    d3 = params.dist(3);
    d4 = params.dist(4);
    d5 = params.dist(5);
    d6 = params.dist(6);

    %--- Translational kinematics ---
    dx(1:3) = [vx; vy; vz];

    %--- Translational dynamics (Eq.1, first 3 lines) ---
    dx(4) = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * (T/params.m) + d1;
    dx(5) = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * (T/params.m) + d2;
    dx(6) = (cos(phi)*cos(theta)) * (T/params.m) - params.g        + d3;

    %--- Compute body moments from thrust differences ---
    Mx = params.L*(tau4 - tau2);                          % roll
    My = params.L*(tau3 - tau1);                          % pitch
    Mz = params.c_tau*(tau1 - tau2 + tau3 - tau4);        % yaw

    %--- Rotational dynamics (Eq.1, last 3 lines) ---
    dx(10) = ((params.Iy - params.Iz)*q*r + Mx   - params.JR*q*params.OmegaR) / params.Ix + d4;
    dx(11) = ((params.Iz - params.Ix)*p*r + My   + params.JR*p*params.OmegaR) / params.Iy + d5;
    dx(12) = ((params.Ix - params.Iy)*p*q + Mz)                 / params.Iz + d6;

    %--- Euler‐angle kinematics ---
    dx(7) = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    dx(8) = cos(phi)*q - sin(phi)*r;
    dx(9) = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;
end
