function eul = rotm2eul_custom(R, sequence)
    % Converts a rotation matrix to 3-2-1 (ZYX) Euler angles
    if nargin < 2
        sequence = 'ZYX';
    end

    if strcmpi(sequence, 'ZYX')
        % Extract ZYX (yaw-pitch-roll) Euler angles from rotation matrix
        if abs(R(3,1)) < 1
            theta = -asin(R(3,1));
            psi = atan2(R(3,2)/cos(theta), R(3,3)/cos(theta));
            phi = atan2(R(2,1)/cos(theta), R(1,1)/cos(theta));
        else
            % Gimbal lock
            phi = 0;
            if R(3,1) <= -1
                theta = pi/2;
                psi = atan2(R(1,2), R(1,3));
            else
                theta = -pi/2;
                psi = atan2(-R(1,2), -R(1,3));
            end
        end
        eul = [phi, theta, psi];  % roll, pitch, yaw
    else
        error('Only ZYX (3-2-1) sequence is supported');
    end
end
