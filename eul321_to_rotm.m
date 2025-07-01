function R = eul321_to_rotm(eul)
    % Convert [roll pitch yaw] ZYX to rotation matrix
    phi = eul(1); theta = eul(2); psi = eul(3);
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
               0         0   1];
    Ry = [cos(theta) 0 sin(theta);
                 0   1     0;
         -sin(theta) 0 cos(theta)];
    Rx = [1     0          0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];
    R = Rz * Ry * Rx;
end