% Aero8560 Assignment 3
% More functions


% State Rates - Compute time derivatives of each state variable

function Xdot = StateRates(X,FlightData,h,U)

g=FlightData.Inertial.g;
m=FlightData.Inertial.m;

% separate out the current state
u = X(1);
v = X(2);
w = X(3);

p = X(4);
q = X(5);
r = X(6);

quaternions = X(7:10);
q0 = quaternions(1);
q1 = quaternions(2);
q2 = quaternions(3);
q3 = quaternions(4);

E = q2e(quaternions);
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

error=1; % a nice random big number
tol = 10^(-8);

% AngularRates_approx to something friendly
AngularRates_approx = [0,0];
Xdot = zeros(13,1);


Xdot(11:13) = DCM(quaternions)*[u; v; w];

% The iteration loop starts here!
while error > tol
    % Find the body forces
    [body, M, N, L] = BodyForces(FlightData, X, AngularRates_approx,h, U);
    Thrust = PropForce(FlightData, X, h, U);

    Fx=body(1)+Thrust;
    Fy=body(2);
    Fz=body(3);

    % Find the state rates
    udot = r*v-q*w-g*sin(theta)+Fx/m;
    vdot = -r*u+p*w+g*sin(phi)*cos(theta)+Fy/m;
    wdot = q*u-p*v+g*cos(phi)*cos(theta)+Fz/m;

    pdot = C3*p*q+C4*q*r+C1*L+C2*N;
    qdot = C7*p*r-C6*(p^2-r^2)+C5*M;
    rdot = C9*p*q-C3*q*r+C2*L+C8*N;

    q0dot = -0.5*(q1*p+q2*q+q3*r);
    q1dot = 0.5*(q0*p-q3*q+q2*r);
    q2dot = 0.5*(q3*p+q0*q-q1*r);
    q3dot = -0.5*(q2*p-q1*q-q0*r);

    XYZdot = DCM(quaternions)*[u; v; w];

Xdot = [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;XYZdot];

    AngularRates_calc=AngularRates(Xdot);
    error=sum(abs(AngularRates_approx-AngularRates_calc));
    AngularRates_approx=AngularRates_calc;

end


end



