% Flight Mech 1 - Assignment 1
% Task 3: Do you gotta fix that attitude!

% Function converting Euler angles to quaternions
% Required to give a 3x1 and spit out a 4x1

function vec = e2q(EulerInDegs)

phi = deg2rad(EulerInDegs(1));
theta = deg2rad(EulerInDegs(2));
psi = deg2rad(EulerInDegs(3));

q0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
q1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
q2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
q3 = -cos(psi/2)*sin(theta/2)*sin(phi/2) + sin(psi/2)*cos(theta/2)*cos(phi/2);

n = q0^2+q1^2+q3^2+q2^2;

vec = 1/sqrt(n)*[q0;q1;q2;q3]; % normalise it!

end