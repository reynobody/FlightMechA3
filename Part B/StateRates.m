% Aero8560 Assignment 3
% More functions


% State Rates - Compute time derivatives of each state variable

function Xdot = StateRates(X,FlightData,h,U)

% separate out the current state
u = X(1);
v = X(2);
w = X(3);

p = X(4);
q = X(5);
r = X(6);

Q = X(7:10);
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

E = q2e(Q);
phi = E(1);
theta = E(2);
psi = E(3);

Iyy = FlightData.Inertial.Iyy;
Ixx = FlightData.Inertial.Ixx;
Izz = FlightData.Inertial.Izz;
Ixz = FlightData.Inertial.Ixz;

C0 = Ixx*Izz-Ixz^2;
C1 = Izz/C0;
C2 = Ixz/C0;
C3 = C2*(Ixx-Iyy+Izz);
C4 = C1*(Iyy-Izz)-C2*Ixz;
C5 = 1/Iyy;
C6 = C5*Ixz;
C7 = C5*(Izz-Ixx);
C8 = Ixx/C0;
C9 = C8*(Ixx-Iyy)+C2*Ixz;

% Find the body forces
Fx = ;
Fy = ;
Fz= ;
L = ;
M = ;
N = ;
BodyForces

% Find the state rates
Xdot(1) = r*v-q*w-g*sin(theta)+Fx/m;
Xdot(2) = -r*u+p*w+g*sin(phi)*cos(theta)+Fy/m;
Xdot(3) = q*u-p*v+g*cos(phi)*cos(theta)+Fz/m;

Xdot(4) = C3*p*q+C4*q*r+C1*L+C2*N;
Xdot(5) = C7*p*r-C6*(p^2-r^2)+C5*M;
Xdot(6) = C9*p*q-C3*q*r+C2*L+C8*N;

Xdot(7) = -0.5*(q1*p+q2*q+q3*r);
Xdot(8) = 0.5*(q0*p-q3*q+q2*r);
Xdot(9) = 0.5*(q3*p+q0*q-q1*r);
Xdot(10) = -0.5*(q2*p-q1*q-q0*r);

Xdot(11:13) = DCM(Q)*[u, v, w]';


end



