function ref = generate_reference(t)
    % generate_reference  Straight‐line ref from p0 to pf over time vector t
    %
    %   ref.px,ref.py,ref.pz : positions
    %   ref.vx,ref.vy,ref.vz : velocities
    %   ref.yaw             : yaw angle (kept zero here)

    % Define start and end positions (you can change these)
    p0 = [0; 0; 1];        % start at [x0;y0;z0]
    pf = [5; 5; 1];        % end   at [xf;yf;zf]

    t0 = t(1);
    tf = t(end);

    % Normalized time on [0,1]
    tau = (t - t0) / (tf - t0);

    % Position: p_ref(t) = p0 + (pf - p0)*tau
    ref.px = p0(1) + (pf(1)-p0(1)) * tau;
    ref.py = p0(2) + (pf(2)-p0(2)) * tau;
    ref.pz = p0(3) + (pf(3)-p0(3)) * tau;

    % Velocity: constant v = (pf - p0)/(tf - t0)
    v_const = (pf - p0) / (tf - t0);
    ref.vx =  v_const(1) * ones(size(t));
    ref.vy =  v_const(2) * ones(size(t));
    ref.vz =  v_const(3) * ones(size(t));

    % Keep yaw zero
    ref.yaw = zeros(size(t));
end