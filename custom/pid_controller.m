function u = pid_controller(ref, x_est, params, gains, dt)
    % PID Controller for Quadrotor
    % ref:   struct with px,py,pz,yaw (scalars)
    % x_est: 12×1 [px;py;pz; vx;vy;vz; phi;theta;psi; p;q;r]
    % dt:    timestep

    persistent Ierr Perr
    if isempty(Ierr)
        Ierr = zeros(6,1);    % [ex; ey; ez; ephi; etheta; eyaw]
        Perr = zeros(6,1);
    end

    %--- Extract estimated states ---
    px    = x_est(1); 
    py    = x_est(2); 
    pz    = x_est(3);
    phi   = x_est(7); 
    theta = x_est(8); 
    psi   = x_est(9);

    %--- Compute error vector [pos; attitude] ---
    err = [ ref.px    - px;
            ref.py    - py;
            ref.pz    - pz;
            0         - phi;   % desired roll = 0
            0         - theta; % desired pitch = 0
            ref.yaw   - psi ];

    %--- Integrator & derivative terms ---
    Ierr = Ierr + err * dt;
    Derr = (err - Perr) / dt;
    Perr = err;

    %--- Outer loop: desired total force in body-z ---
    Fdes = gains.Kp_pos(:) .* err(1:3) + ...
           gains.Ki_pos(:) .* Ierr(1:3) + ...
           gains.Kd_pos(:) .* Derr(1:3);

    %--- Inner loop: attitude torques ---
    tau_phi   = gains.Kp_att(1)*err(4) + gains.Ki_att(1)*Ierr(4) + gains.Kd_att(1)*Derr(4);
    tau_theta = gains.Kp_att(2)*err(5) + gains.Ki_att(2)*Ierr(5) + gains.Kd_att(2)*Derr(5);
    tau_psi   = gains.Kp_att(3)*err(6) + gains.Ki_att(3)*Ierr(6) + gains.Kd_att(3)*Derr(6);

    %--- Thrust (body-z) with gravity compensation ---
    T = params.m * params.g + Fdes(3);
    T = max(0, T);  % prevent negative thrust

    %--- Return control inputs [tau_roll; tau_pitch; tau_yaw; thrust] ---
    u = [ tau_phi;
          tau_theta;
          tau_psi;
          T ];
end
