%% === Quaternion to Euler ZYX (3-2-1) ===
function eul = quat2eul321(q)
    w = q(1); x = q(2); y = q(3); z = q(4);
    R11 = 1 - 2*y^2 - 2*z^2;
    R21 = 2*(x*y + z*w);
    R31 = 2*(x*z - y*w);
    R32 = 2*(y*z + x*w);
    R33 = 1 - 2*x^2 - 2*y^2;
    phi = atan2(R32, R33);       % roll
    theta = -asin(R31);          % pitch
    psi = atan2(R21, R11);       % yaw
    eul = [phi; theta; psi];
end
