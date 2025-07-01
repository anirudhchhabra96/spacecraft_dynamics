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
z_offset_cube = 99; % [mm] height of cube center above moving platform

%% === Simulate Spacecraft Rotation ===
tspan = 0:0.05:10;
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
figure;
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

%% === Shutdown ===
PIdevice.MOV('X Y Z U V W', [0, 0, 0, 0, 0, 0]);
while PIdevice.IsMoving(), pause(0.1); end
PIdevice.SVO('X Y Z U V W', [0, 0, 0, 0, 0, 0]);
PIdevice.CloseConnection();
Controller.Destroy();


