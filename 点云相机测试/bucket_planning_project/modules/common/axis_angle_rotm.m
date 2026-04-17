function R = axis_angle_rotm(axisVec, angleRad)
% axis_angle_rotm:
% Rodrigues rotation matrix from axis-angle.

a = axisVec(:);
na = norm(a);
if na < 1e-12 || abs(angleRad) < 1e-12
    R = eye(3);
    return;
end

a = a / na;
x = a(1);
y = a(2);
z = a(3);
c = cos(angleRad);
s = sin(angleRad);
C = 1 - c;

R = [ ...
    x*x*C + c,     x*y*C - z*s, x*z*C + y*s; ...
    y*x*C + z*s, y*y*C + c,     y*z*C - x*s; ...
    z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end

