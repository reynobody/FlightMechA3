% AERO8560 Assignment 3
% Calculate air density and dynamic pressure as a function of altitude and
% airspeed

function [rho Q] = FlowProperties(alt,airspeed) 

% General constants
g = 9.81;       
R = 287.1;          
L = 0.0065;        

% Sea-Level Conditions
P0 = 101.3*10^3;    % Pressure [kPa]
T0  = 288.15;    % Temperature [K]

% Calculate pressures, temperatures, density at given altitude h
T  = T0 - L*alt;
P = P0 * (T/T0)^(g/(L*R));
rho   = P/(R*T); % Ideal gas eqt
Q = 0.5*rho*airspeed^2;

end
