function animate_sim_vs_hexapod(T, X, actual_angles)
figure(2);clf;
set(gcf, 'Color', 'w');



% Cube geometry
[v, f] = create_cube();

% Limits and view settings
% cube_extent = 1.2;
% lim = cube_extent * [-1 1 -1 1 -1 1];
cube_extent = 1.5;
lim = cube_extent * [-1 1 -1 1 -1 1];
text_offset = cube_extent * 0.9;

sgtitle("Simulated Spacecraft Dynamics using Hexapod-based actuation")
% === Left: Simulated Cube ===
subplot(1,2,1);
p1 = patch('Vertices', v, 'Faces', f, ...
    'FaceColor', [1.0 0.84 0], ...
    'EdgeColor', 'k', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, ...
    'DiffuseStrength', 0.6, ...
    'SpecularStrength', 1.0, ...
    'SpecularExponent', 25, ...
    'BackFaceLighting', 'reverselit');
light('Position',[1 2 3],'Style','infinite');
material shiny;
title('Numerical Simulation');
axis(lim); axis vis3d equal; 
view(160,20); view(0,0);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;
rline1 = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 2);
% txt1 = text(-1.2, 1.3, 1.2, '', 'FontSize', 10, 'BackgroundColor', 'w');
txt1 = text(-text_offset, text_offset, text_offset, '', ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');
% Create UI text boxes instead of 3D text in scene
txt1 = annotation('textbox', [0.14 0.05 0.3 0.12], ...
    'String', '', 'FontSize', 10, 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'Interpreter', 'tex');

% === Right: Hexapod Actual Cube ===
subplot(1,2,2);
p2 = patch('Vertices', v, 'Faces', f, ...
    'FaceColor', [1.0 0.84 0], ...
    'EdgeColor', 'k', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.3, ...
    'DiffuseStrength', 0.6, ...
    'SpecularStrength', 1.0, ...
    'SpecularExponent', 25, ...
    'BackFaceLighting', 'reverselit');
light('Position',[1 2 3],'Style','infinite');
material shiny;
title('HIL Simulation');
axis(lim); axis vis3d equal; 
view(160,20); 
view(0,0);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;
rline2 = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 2);
% txt2 = text(-1.2, 1.3, 1.2, '', 'FontSize', 10, 'BackgroundColor', 'w');

txt2 = annotation('textbox', [0.59 0.05 0.3 0.12], ...
    'String', '', 'FontSize', 10, 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'Interpreter', 'tex');

vidobj = VideoWriter('sim_vs_hexapod.mp4', 'MPEG-4');  % or 'Uncompressed AVI'
vidobj.FrameRate = 5;  % Lower = slower playback
open(vidobj);


% === Animation Loop ===
for k = 1:5:length(T)
    % Simulated: quaternion → rotation matrix
    q = X(k, 4:7)';
    omega_sim = X(k, 1:3);
    R_sim = quat2rotm(q);
    v_rot_sim = (R_sim * v')';
    set(p1, 'Vertices', v_rot_sim);

    % Axis of rotation (unit omega vector, in inertial frame)
    if norm(omega_sim) > 1e-6
        w_hat = omega_sim / norm(omega_sim);
        w_arrow = R_sim * w_hat(:);
        pts = [-2*w_arrow, 2*w_arrow];
        set(rline1, 'XData', pts(1,:), 'YData', pts(2,:), 'ZData', pts(3,:));
    end

    % Live text for Sim
    eul_sim = quat2eul321(q);
    % txt1.String = sprintf( ...
    %     'Time: %.1f s\n\\omega = [%.2f %.2f %.2f] deg/s\n\\phi = %.1f° \\theta = %.1f° \\psi = %.1f°', ...
    %     T(k), ...
    %     rad2deg(omega_sim(1)), rad2deg(omega_sim(2)), rad2deg(omega_sim(3)), ...
    %     rad2deg(eul_sim(1)), rad2deg(eul_sim(2)), rad2deg(eul_sim(3)));
    txt1.String = sprintf( ...
        'Time: %.1f s\n\\omega = [%.2f %.2f %.2f] deg/s\n\\phi = %.1f°  \\theta = %.1f°  \\psi = %.1f°', ...
        T(k), ...
        rad2deg(omega_sim(1)), rad2deg(omega_sim(2)), rad2deg(omega_sim(3)), ...
        rad2deg(eul_sim(1)), rad2deg(eul_sim(2)), rad2deg(eul_sim(3)));



    % Actual: Euler angles → rotation matrix
    rpy_deg = actual_angles(k, :);
    R_real = eul321_to_rotm(deg2rad(rpy_deg));
    v_rot_real = (R_real * v')';
    set(p2, 'Vertices', v_rot_real);

    % Axis of rotation for hexapod (approximate using finite diff)
    if k > 1
        dR = R_real * eul321_to_rotm(deg2rad(actual_angles(k-1,:)))';
        axis_angle = rotm_to_axis_angle(dR);
        w_real = axis_angle.axis * axis_angle.angle / (T(k) - T(k-1));
        w_hat2 = w_real / norm(w_real);
        % pts2 = [-2*w_hat2; 2*w_hat2]';
        pts2 = [-2*w_hat2, 2*w_hat2];  % Result: 3×2 matrix
        set(rline2, 'XData', pts2(1,:), 'YData', pts2(2,:), 'ZData', pts2(3,:));
    end

    % Live text for Hexapod
    % txt2.String = sprintf( ...
    %     'Time: %.1f s\n\\phi = %.1f° \\theta = %.1f° \\psi = %.1f°', ...
    %     T(k), ...
    %     rpy_deg(1), rpy_deg(2), rpy_deg(3));

    txt2.String = sprintf( ...
        'Time: %.1f s\n\\phi = %.1f°  \\theta = %.1f°  \\psi = %.1f°', ...
        T(k), ...
        rpy_deg(1), rpy_deg(2), rpy_deg(3));
    drawnow;
    % === Capture frame as image and write to GIF ===
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [imind, cm] = rgb2ind(im, 256);

    % if k == 1
    %     imwrite(imind, cm, 'sim_vs_hexapod.gif', ...
    %         'gif', 'Loopcount', inf, 'DelayTime', 0.2);  % first frame
    % else
    %     imwrite(imind, cm, 'sim_vs_hexapod.gif', ...
    %         'gif', 'WriteMode', 'append', 'DelayTime', 0.2);  % 0.2s per frame
    % end
    frame = getframe(gcf);
    writeVideo(vidobj, frame);


    pause(0.2);
end
close(vidobj);
disp('Video saved as sim_vs_hexapod.mp4');
end
