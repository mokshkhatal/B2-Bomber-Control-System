clear; clc;

initialization = 1;

if initialization == 0
    Z0 = zeros(17,1);

    Z0(1) = 85;    % u (body x-velocity)
    Z0(2) = 0;     % v (body y-velocity)
    Z0(3) = 0;     % w (body z-velocity)
    Z0(4:6) = 0;   % angular rates p,q,r
    Z0(7) = 0;     % bank angle phi
    Z0(8) = 0;     % pitch angle theta
    Z0(9) = 0;     % heading angle psi
    Z0(10:12) = 0; % position

    Z0(13:17) = [0; 0; 0; 5*pi/180; 5*pi/180];  % initial controls

else
    load('trim_values_6dof.mat', 'Z_trim');
    Z0 = Z_trim;
end

% Bounds for states
lb_states = [-inf; -1; -inf; -0.2; -0.2; -0.2; -pi/12; -pi/12; -pi; -inf; -inf; -inf];
ub_states = [inf; 1; inf; 0.2; 0.2; 0.2; pi/12; pi/12; pi; inf; inf; inf];

% Bounds for controls (in radians)
lb_controls = [-30*pi/180; -30*pi/180; -30*pi/180; 2*pi/180; 2*pi/180];
ub_controls = [30*pi/180; 30*pi/180; 30*pi/180; 12*pi/180; 12*pi/180];

lb = [lb_states; lb_controls];
ub = [ub_states; ub_controls];

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter', ...
    'MaxIterations', 2000, ...
    'MaxFunctionEvaluations', 2e5, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-8);

[Z_trim, cost_val] = fmincon(@cost_straight_level_6dof_october, Z0, [], [], [], [], lb, ub, [], options);

X_trim = Z_trim(1:12);
U_trim = Z_trim(13:17);

Va = sqrt(X_trim(1)^2 + X_trim(2)^2 + X_trim(3)^2);
alpha = atan2(X_trim(3), X_trim(1));
gam = X_trim(8) - alpha;

fprintf('\nTrim Results:\n');
fprintf('Airspeed (Va): %.5f m/s\n', Va);
fprintf('Flight Path Angle (gam): %.5f rad\n', gam);
fprintf('Sideslip Velocity (v): %.5f m/s\n', X_trim(2));
fprintf('Bank Angle (phi): %.5f rad\n', X_trim(7));
fprintf('Heading Angle (psi): %.5f rad\n', X_trim(9));
fprintf('Angular rates (p, q, r): %.5f, %.5f, %.5f rad/s\n', X_trim(4), X_trim(5), X_trim(6));
fprintf('Control inputs (u1, u2, u3, u4, u5) [rad]: %.5f, %.5f, %.5f, %.5f, %.5f\n', U_trim);

% Extract trimmed state and control vectors
X_trim = Z_trim(1:12);
U_trim = Z_trim(13:17);

% Display all 17 values together
fprintf('\n=== Full Trim State and Control Vector ===\n');
for i = 1:12
    fprintf('x%d = %.6f\n', i, X_trim(i));
end
for i = 1:5
    fprintf('u%d = %.6f\n', i, U_trim(i));
end

save('trim_values_6dof.mat', 'Z_trim');
