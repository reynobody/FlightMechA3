% AERO8560 -  A3
% Integration function: 4th order Runge Kutta
% Not allowed to use inbuilt matlab functions

function Xnew = Integrate(X,FlightData,h,U,dt)
% Step 1:
xdot1 = StateRates(X,FlightData,h,U);
An = xdot1*dt;

% Step 2:
xdot2 = StateRates(X+An/2,FlightData,h,U);
Bn = xdot2*dt;

% Step 3:
xdot3 = StateRates(X+Bn/2,FlightData,h,U);
Cn = xdot3*dt;

% Step 4:
xdot4 = StateRates(X+Cn,FlightData,h,U);
Dn = xdot4*dt;

% Step 5:
Xnew = X+dt/6*(xdot1+2*xdot2+2*xdot3+xdot4);

end