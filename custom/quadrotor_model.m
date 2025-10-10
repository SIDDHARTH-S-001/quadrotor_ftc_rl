%% Quadrotor dynamics (with velocity & acceleration limits)
function dx = quadrotor_model(~, x, u, params)
    % quadrotor_model  12-state dynamics for a +-quadrotor
    % x = [px;py;pz; vx;vy;vz; phi;theta;psi; p;q;r]
    % u = [tau_phi; tau_theta; tau_psi; thrust]
    % params.dist = [d1…d6], params.v_max, params.a_max

    % Preallocate
    dx = zeros(12,1);

    % Unpack states
    vx    = x(4); vy    = x(5); vz    = x(6);
    phi   = x(7); theta = x(8); psi   = x(9);
    p     = x(10); q    = x(11); r     = x(12);

    % Unpack inputs
    tau_phi   = u(1);
    tau_theta = u(2);
    tau_psi   = u(3);
    T         = u(4);

    % Unpack disturbances
    d1 = params.dist(1); d2 = params.dist(2); d3 = params.dist(3);
    d4 = params.dist(4); d5 = params.dist(5); d6 = params.dist(6);

    %--- Translational kinematics (velocities) ---
    dx(1) = vx;
    dx(2) = vy;
    dx(3) = vz;

    %--- Translational dynamics ---
    % total thrust T already includes gravity compensation from PID
    ax = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * (T/params.m) + d1;
    ay = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * (T/params.m) + d2;
    az = (cos(phi)*cos(theta)) * (T/params.m) - params.g        + d3;
    dx(4) = ax;
    dx(5) = ay;
    dx(6) = az;

    %--- Rotational kinematics (Euler angles rates) ---
    dx(7)  = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    dx(8)  = cos(phi)*q - sin(phi)*r;
    dx(9)  = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;

    %--- Rotational dynamics ---
    % convert back to individual rotor-thrust conventions
    Mx = tau_phi;    % roll moment
    My = tau_theta;  % pitch moment
    Mz = tau_psi;    % yaw moment

    dx(10) = ((params.Iy - params.Iz)*q*r + Mx   - params.JR*q*params.OmegaR) / params.Ix + d4;
    dx(11) = ((params.Iz - params.Ix)*p*r + My   + params.JR*p*params.OmegaR) / params.Iy + d5;
    dx(12) = ((params.Ix - params.Iy)*p*q + Mz)                 / params.Iz + d6;

    %--- Enforce velocity limits on dx(1:3) ---
    for k = 1:3
        dx(k) = sign(dx(k)) * min(abs(dx(k)), params.v_max(k));
    end

    %--- Enforce acceleration limits on dx(4:6) ---
    for k = 1:3
        dx(3+k) = sign(dx(3+k)) * min(abs(dx(3+k)), params.a_max(k));
    end
end