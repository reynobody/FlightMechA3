% Flight Mech 1 - Assignment 1
% Task 3: Do you gotta fix that attitude!

% Function converting quaternions to Euler angles
% Required to give a 4x1 and spit out a 3x1

function vec = q2e(quaternions) 

q0 = quaternions(1);
q1 = quaternions(2);
q2 = quaternions(3);
q3 = quaternions(4);

theta = atan2((q0*q2 - q1*q3),((q0^2+q1^2-0.5)^2 + (q1*q2+q0*q3)^2)^0.5);
phi = atan2((q2*q3+q0*q1),(q0^2+q3^2-0.5));
psi = atan2((q1*q2+q0*q3),(q0^2+q1^2-0.5));

vec = rad2deg([phi theta psi]');
end