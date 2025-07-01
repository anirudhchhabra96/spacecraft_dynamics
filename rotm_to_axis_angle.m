function result = rotm_to_axis_angle(R)
% Converts a 3x3 rotation matrix into axis-angle representation
% Returns:
%   result.axis  = unit vector (3x1)
%   result.angle = rotation angle in radians

angle = acos((trace(R) - 1) / 2);

% Handle numerical issues near 0 rotation
if abs(angle) < 1e-6
    axis = [1; 0; 0];  % default axis
else
    axis = 1/(2*sin(angle)) * [R(3,2) - R(2,3);
                               R(1,3) - R(3,1);
                               R(2,1) - R(1,2)];
end

result.axis = axis / norm(axis);
result.angle = angle;
end
