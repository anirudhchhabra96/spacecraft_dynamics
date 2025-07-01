clear; 
% close all; 
clc;

% Time span
tspan = 0:0.1:100;

% Initial angular velocity (rad/s)
omega0 = [0.001; 0.001; 0.01];  % body frame

% Initial quaternion (identity, no rotation)
q0 = [1; 0; 0; 0];  % [w x y z]

% Initial state: [omega; quaternion]
x0 = [omega0; q0];

% Inertia matrix
J = diag([1.0, 1.5, 2.0]);

options = odeset('RelTol', 1e-14, 'AbsTol', 1e-18);
% Simulate
[T, X] = ode45(@(t, x) spacecraft_dynamics(t, x, J), tspan, x0, options);
% Extract angular velocity from state vector
omega_body = X(:, 1:3);  % Each row is [ωx ωy ωz] at a time step

% Convert to degrees/sec for plotting (optional)
omega_deg = rad2deg(omega_body);



% Extract Euler angles over time
euler_angles = zeros(length(T), 3);
for i = 1:length(T)
    q = X(i,4:7)';
    euler_angles(i,:) = quat2eul321(q)';
end
% Compute Euler angle rates from omega and Euler angles
euler_rates = zeros(size(euler_angles));
for i = 1:length(T)
    phi = euler_angles(i,1);
    theta = euler_angles(i,2);
    omega = X(i,1:3)';

    % Transformation matrix T_inv
    T_inv = [
        1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi),           -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)
        ];

    euler_rates(i,:) = T_inv * omega;
end

% Convert to degrees/sec
euler_rates_deg = rad2deg(euler_rates);


% Convert radians to degrees for plotting
euler_deg = rad2deg(euler_angles);


% Compute y-limits with 20% margin for each plot
pad = @(data) 1.2 * max(abs(data), [], 'all');
y1_lim = pad(omega_deg);
y2_lim = pad(euler_rates_deg);
y3_lim = pad(euler_deg);

% Create figure with 3 subplots
figure(1); clf;
% --- Subplot 1: Angular Velocity ---
subplot(3,1,1);
plot(T, omega_deg, 'LineWidth', 1.2);
ylabel('Angular Velocity (deg/s)');
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'northeast');
title('Body Angular Velocity', 'Interpreter','latex');
ylim([-y1_lim, y1_lim]);
grid on;

% --- Subplot 2: Euler Angle Rates ---
subplot(3,1,2);
plot(T, euler_rates_deg, 'LineWidth', 1.2);
ylabel('Euler Rate (deg/s)', 'Interpreter', 'latex');
legend({'$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$'}, ...
       'Interpreter','latex', 'Location','northeast');
title('Euler Angle Rates (ZYX)', 'Interpreter','latex');
ylim([-y2_lim, y2_lim]);
grid on;

% --- Subplot 3: Euler Angles ---
subplot(3,1,3);
plot(T, euler_deg, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Euler Angle (deg)', 'Interpreter', 'latex');
legend({'$\phi$ (Roll)', '$\theta$ (Pitch)', '$\psi$ (Yaw)'}, ...
       'Interpreter','latex', 'Location','northeast');
title('Euler Angles (3-2-1 ZYX)', 'Interpreter','latex');
ylim([-y3_lim, y3_lim]);
grid on;



% Animate
animate_cube(T, X);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = spacecraft_dynamics(~, x, J)
omega = x(1:3);  % Angular velocity in body frame
q = x(4:7);      % Quaternion [w x y z]

% Euler's equations: dω/dt = J⁻¹ ( -ω × (Jω) )
domega = J \ ( -cross(omega, J * omega) );

% Quaternion kinematics: dq/dt = 0.5 * Ω(ω) * q
w = omega(1); u = omega(2); v = omega(3);
Omega = 0.5 * [  0   -w  -u  -v;
    w    0   v  -u;
    u  -v    0   w;
    v   u  -w    0];
dq = Omega * q;

dx = [domega; dq];
end
function animate_cube(T, X)
    figure(2); clf;
    axis equal;
    axis([-1 1 -1 1 -1 1]*2);
    view(-160,20);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    % title('Spacecraft Attitude (Cube)');
    set(gca, 'Projection', 'perspective');

    % Define cube
    [v, f] = create_cube();

    % Gold material and lighting
    p = patch('Vertices', v, 'Faces', f, ...
              'FaceColor', [1.0 0.84 0], ...         % Gold color (RGB)
              'EdgeColor', 'k', ...
              'FaceLighting', 'gouraud', ...
              'AmbientStrength', 0.3, ...
              'DiffuseStrength', 0.6, ...
              'SpecularStrength', 1.0, ...
              'SpecularExponent', 25, ...
              'BackFaceLighting', 'reverselit');

    light('Position', [1 2 3], 'Style', 'infinite');
    material shiny;

    hold on;

    % Axis of rotation line (bidirectional)
    axis_line = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 2);

    % Live text
    txt = text(-1.8, 1.5, 1.8, '', 'FontSize', 10, ...
               'BackgroundColor', 'w', 'EdgeColor', 'k');

    for k = 1:10:length(T)
        q = X(k, 4:7)';            % Quaternion
        omega = X(k, 1:3)';        % Angular velocity (body frame)
        R = quat2rotm(q);          % Rotation matrix

        % Rotate cube
        v_rot = (R * v')';
        set(p, 'Vertices', v_rot);

        % Axis of rotation line (bidirectional)
        if norm(omega) > 1e-6
            w_hat = omega / norm(omega);     % Unit vector
            w_arrow = R * w_hat;             % In inertial frame
            scale = 2;                       % Arrow length
            pts = [-scale * w_arrow, scale * w_arrow];  % Two-way
            set(axis_line, 'XData', pts(1,:), ...
                           'YData', pts(2,:), ...
                           'ZData', pts(3,:));
        end

        % Live text
        eul = quat2eul321(q); % [phi; theta; psi]
        txt.String = sprintf( ...
            'Time: %.1f s\n\\omega = [%.2f, %.2f, %.2f] deg/s\n\\phi = %.1f°, \\theta = %.1f°, \\psi = %.1f°', ...
            T(k), ...
            rad2deg(omega(1)), rad2deg(omega(2)), rad2deg(omega(3)), ...
            rad2deg(eul(1)), rad2deg(eul(2)), rad2deg(eul(3)));

        drawnow;
        pause(0.1);
    end
end



function [v, f] = create_cube()
% Define unit cube centered at origin
v = 0.5 * [ ...
    -1 -1 -1;
    1 -1 -1;
    1  1 -1;
    -1  1 -1;
    -1 -1  1;
    1 -1  1;
    1  1  1;
    -1  1  1];
f = [ ...
    1 2 3 4;  % Bottom
    5 6 7 8;  % Top
    1 2 6 5;  % Side
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];
end

function R = quat2rotm(q)
% Convert quaternion [w x y z] to rotation matrix
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*y^2 - 2*z^2,     2*x*y - 2*z*w,     2*x*z + 2*y*w;
    2*x*y + 2*z*w, 1 - 2*x^2 - 2*z^2,     2*y*z - 2*x*w;
    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x^2 - 2*y^2];
end

function eul = quat2eul321(q)
% Convert quaternion [w x y z] to 3-2-1 (ZYX) Euler angles
w = q(1); x = q(2); y = q(3); z = q(4);

% Rotation matrix elements
R11 = 1 - 2*y^2 - 2*z^2;
R21 = 2*(x*y + z*w);
R31 = 2*(x*z - y*w);
R32 = 2*(y*z + x*w);
R33 = 1 - 2*x^2 - 2*y^2;

% ZYX Euler angles
phi   = atan2(R32, R33);        % roll
theta = -asin(R31);             % pitch
psi   = atan2(R21, R11);        % yaw

eul = [phi; theta; psi];
end
