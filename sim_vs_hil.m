clear all; clc;

%% === PI Hexapod Setup ===
ip = '192.168.20.3';
port = 50000;
addpath(getenv('PI_MATLAB_DRIVER'));

% Load driver
if ~exist('Controller', 'var') || ~isa(Controller, 'PI_GCS_Controller')
    Controller = PI_GCS_Controller();
end

% Connect to device
if ~exist('PIdevice', 'var') || ~PIdevice.IsConnected
    PIdevice = Controller.ConnectTCPIP(ip, port);
end
PIdevice = PIdevice.InitializeController();

% Servo on
PIdevice.SVO('X Y Z U V W', [1 1 1 1 1 1]);
pause(0.5);

% Safe home pose
PIdevice.MOV('X Y Z U V W', [0, 0, 0, 0, 0, 0]);
while PIdevice.IsMoving(), pause(0.1); end

% Velocity limit
PIdevice.VLS(20);  % deg/s or mm/s

% Constants
z_offset_cube = 99.12; % [mm] height of cube center above moving platform

%% === Simulate Spacecraft Rotation ===
tspan = 0:0.05:50;
omega0 = [0.001; 0.001; 0.01];     % initial angular velocity
q0 = [1; 0; 0; 0];                 % identity quaternion
x0 = [omega0; q0];
J = diag([1.0, 1.5, 2.0]);

opts = odeset('RelTol', 1e-14, 'AbsTol', 1e-18);
[T, X] = ode45(@(t, x) spacecraft_dynamics(t, x, J), tspan, x0, opts);

% Desired Euler angles (ZYX: roll-pitch-yaw)
euler_des_rad = zeros(length(T), 3);
for i = 1:length(T)
    q = X(i, 4:7)';
    euler_des_rad(i,:) = quat2eul321(q)';  % [φ θ ψ]
end
euler_des_deg = rad2deg(euler_des_rad);

%% === Command Hexapod ===
fprintf('Streaming trajectory to hexapod...\n');
actual_angles = zeros(length(T), 3);  % store [roll pitch yaw]

for i = 1:length(T)
    % Desired platform rotation that causes cube to match simulated cube rotation
    % Because cube is offset in Z, we need to rotate *platform* accordingly
    roll = euler_des_deg(i, 1);
    pitch = euler_des_deg(i, 2);
    yaw = euler_des_deg(i, 3);

    % Send platform pose [X Y Z U V W]
    % X/Y/Z = 0; U/V/W = roll/pitch/yaw
    PIdevice.MOV('X Y Z U V W', [0, 0, 0, roll, pitch, yaw]);

    % Wait for movement (or use time step)
    while PIdevice.IsMoving()
        pause(0.01);
    end

    % Read actual pose (in deg)
    current_pose = PIdevice.qPOS('U V W');
    actual_angles(i, :) = current_pose;

    % Wait for next step
    if i < length(T)
        pause(T(i+1) - T(i));
    end
end

fprintf('Trajectory complete.\n');
animate_sim_vs_hexapod(T, X, actual_angles);

%% === Plot Desired vs Actual Euler Angles ===
figure(1); clf;
titles = {'Roll (°)', 'Pitch (°)', 'Yaw (°)'};
for j = 1:3
    subplot(3,1,j);
    plot(T, euler_des_deg(:,j), 'b-', 'LineWidth', 1.5); hold on;
    plot(T, actual_angles(:,j), 'r--', 'LineWidth', 1.2);
    ylabel(titles{j});
    legend('Desired', 'Actual');
    grid on;
end
xlabel('Time (s)');
sgtitle('Hexapod Tracked vs Desired Cube Attitude');

%%
euler_sim = zeros(length(T), 3);   % radians
for k = 1:length(T)
    q = X(k, 4:7);
    euler_sim(k, :) = rotm2eul_custom(quat_to_rotm(q), 'ZYX');  % [phi theta psi]
end
euler_sim_deg = rad2deg(euler_sim);

euler_dot_sim = zeros(length(T), 3);  % rad/s
for k = 1:length(T)
    phi = euler_sim(k,1);
    theta = euler_sim(k,2);
    omega = X(k, 1:3)';  % body angular velocity (rad/s)

    T_inv = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
             0  cos(phi)             -sin(phi);
             0  sin(phi)/cos(theta)  cos(phi)/cos(theta)];

    euler_dot_sim(k,:) = (T_inv * omega)';  % phi_dot, theta_dot, psi_dot
end
euler_dot_sim_deg = rad2deg(euler_dot_sim);
omega_sim_deg = rad2deg(X(:,1:3));  % ω in deg/s
dt = mean(diff(T));
euler_dot_hex = [zeros(1,3); diff(actual_angles) / dt];  % deg/s
figure;

% === Euler Angles ===
subplot(3,1,1);
plot(T, euler_sim_deg, '-o', T, actual_angles, '-');
ylabel('Euler Angles (deg)');
legend('$\psi$ Sim', '$\theta$ Sim', '$\phi$ Sim', '$\psi$ Hex', '$\theta$ Hex', '$\phi$ Hex','Interpreter','latex');
title('Euler Angle Comparison');

% === Euler Angle Rates ===
subplot(3,1,2);
plot(T, euler_dot_sim_deg, '-o', T, euler_dot_hex, '-');
ylabel('Euler Rates (deg/s)');
legend('$\dot{\phi}$ Sim', '$\dot{\theta}$ Sim', '$\dot\psi$ Sim', '$\dot{\phi}$ Hex', '$\dot{\theta}$ Hex', '$\dot\psi$ Hex','Interpreter','latex');
title('Euler Rate Comparison');

% === Body Angular Velocities ===
subplot(3,1,3);
plot(T, omega_sim_deg, '--');
ylabel('\omega (deg/s)');
xlabel('Time (s)');
legend('\omega_x', '\omega_y', '\omega_z');
title('Simulated Body Angular Velocity');

grid on;



%% === Shutdown ===
PIdevice.MOV('X Y Z U V W', [0, 0, 0, 0, 0, 0]);
while PIdevice.IsMoving(), pause(0.1); end
PIdevice.SVO('X Y Z U V W', [0, 0, 0, 0, 0, 0]);
PIdevice.CloseConnection();
Controller.Destroy();
%%
function R = quat_to_rotm(q)
    % Ensure unit quaternion and format [w x y z]
    if size(q,2) ~= 4
        error('quat_to_rotm expects [w x y z] format');
    end
    q = q / norm(q);  % normalize
    w = q(1); x = q(2); y = q(3); z = q(4);

    R = [ 1 - 2*(y^2 + z^2),     2*(x*y - z*w),     2*(x*z + y*w);
              2*(x*y + z*w), 1 - 2*(x^2 + z^2),     2*(y*z - x*w);
              2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x^2 + y^2) ];
end

