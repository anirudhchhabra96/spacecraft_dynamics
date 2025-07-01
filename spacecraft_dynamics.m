%% === Dynamics Function ===
function dx = spacecraft_dynamics(~, x, J)
    omega = x(1:3);  % body angular velocity
    q = x(4:7);      % quaternion [w x y z]

    domega = J \ (-cross(omega, J*omega));
    Omega = 0.5 * [  0   -omega(1) -omega(2) -omega(3);
                    omega(1)  0    omega(3) -omega(2);
                    omega(2) -omega(3) 0    omega(1);
                    omega(3)  omega(2) -omega(1) 0];
    dq = Omega * q;
    dx = [domega; dq];
end
