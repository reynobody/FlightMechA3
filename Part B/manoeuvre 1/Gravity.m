
% AERO8560 Assignment 3

% Force due to gravity and rotate it to body axes

function vec = Gravity(quaternions)

g = 9.81;
vec = DCM(quaternions)*[0;0;g];

end