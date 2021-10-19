% AERO8560 - Assignment 3

% Compute V, alpha and beta using current body velocities u,v and w
% gives the values in m/s and in radians for the angles

function [V alpha beta] = AeroAngles(u,v,w)

V = sqrt(u^2+v^2+w^2);
alpha = atan(w/u);
beta = asin(v/V);

end