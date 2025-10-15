function cost = cost_straight_level_6dof_october(Z)
    Va_desired = 85;  % desired airspeed [m/s]

    % Extract states and controls
    X = Z(1:12);
    U = Z(13:17);

    % States: u, v, w, p, q, r, phi, theta, psi, x, y, z
    u = X(1);
    v = X(2);
    w = X(3);
    p = X(4);
    q = X(5);
    r = X(6);
    phi = X(7);
    theta = X(8);
    psi = X(9);

    % Compute airspeed and flight path angle
    Va = sqrt(u^2 + v^2 + w^2);
    alpha = atan2(w, u);
    gam = theta - alpha;

    % Residual from 6DOF model dynamics
    Xdot = B2_6DOF_model(X, U);  % residual vector (should be near zero)
    res_cost = sum(Xdot.^2);

    % Control deflection penalty (normalized)
    max_deflection = 30 * pi / 180;
    control_penalty = 0.1 * sum((U / max_deflection).^2);

    % Throttle penalty if below min (2 deg)
    throttle_penalty = 5 * ((U(4) < 2*pi/180) + (U(5) < 2*pi/180));

    % Weights (increase airspeed and sideslip penalties)
    w_va = 1000;         % strong penalty on airspeed deviation
    w_gam = 500;         % flight path angle near zero
    w_chi = 20;          % course angle penalty (optional)
    w_v = 1000;          % strong penalty on sideslip velocity v
    w_phi = 200;         % bank angle penalty to keep small
    w_rates = 100;       % penalty on angular rates
    w_ctrl = 1;          % control usage penalty
    w_dynamics = 10;     % steady-state dynamics penalty

    % Compose cost
    cost = ...
        w_va * (Va - Va_desired)^2 + ...
        w_v * v^2 + ...
        w_gam * gam^2 + ...
        w_phi * phi^2 + ...
        w_rates * (p^2 + q^2 + r^2) + ...
        w_dynamics * res_cost + ...
        w_ctrl * control_penalty + ...
        throttle_penalty;
end
