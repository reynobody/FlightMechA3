% Aero8560 Assignment 3
% More functions


% State Rates - Compute time derivatives of each state variable

function [Xdot,debug] = StateRates(X,FlightData,h,U)

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
E = deg2rad(E);
phi = E(1);
theta = E(2);
psi = E(3);

L_be = DCM(quaternions);
L_eb = inv(DCM(quaternions));

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

% Gravity!
G_body = Gravity(quaternions);
gx = G_body(1);
gy = G_body(2);
gz = G_body(3);

[Thrust] = PropForce(FlightData, X, h, U);

% The iteration loop starts here!
while error > tol
 %for i = 1:3
    % Find the body forces
    [body, L, M, N,debug] = BodyForces(FlightData, X, AngularRates_approx,h, U);


    Fx=body(1)+Thrust;
    Fy=body(2);
    Fz=body(3);

    % Find the state rates
    udot = r*v-q*w+gx+Fx/m;
    vdot = -r*u+p*w+gy+Fy/m;
    wdot = q*u-p*v+gz+Fz/m;

    pdot = C3*p*q+C4*q*r+C1*L+C2*N;
    qdot = C7*p*r-C6*(p^2-r^2)+C5*M;
    rdot = C9*p*q-C3*q*r+C2*L+C8*N;

    q0dot = -0.5*(q1*p+q2*q+q3*r);
    q1dot = 0.5*(q0*p-q3*q+q2*r);
    q2dot = 0.5*(q3*p+q0*q-q1*r);
    q3dot = -0.5*(q2*p-q1*q-q0*r);

    XYZdot = L_eb*[u; v; w];

Xdot = [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;XYZdot];

    AngularRates_calc=AngularRates(X,Xdot);
    error=sum(abs(AngularRates_approx-AngularRates_calc));
    AngularRates_approx=AngularRates_calc;

end

% Chuck in a bunch of debugs here
% Go get the bugs!
debug.Xdot = Xdot;
debug.AngularRates = AngularRates_approx;
debug.Thrust = Thrust;
debug.Gravity = Gravity(quaternions);

debug.Fx  = Fx;
debug.Fy = Fy;
debug.Fz = Fz;
debug.F_body = body;

debug.c0 = C0;
debug.c1 = C1;
debug.c2 = C2;
debug.c3 = C3;
debug.c4 = C4;
debug.c5 = C5;
debug.c6 = C6;
debug.c7 = C7;
debug.c8 = C8;
debug.c9 = C9;

debug.body_vel_rates = [udot;vdot;wdot];
debug.pdot = pdot;
debug.qdot = qdot;
debug.rdot = rdot;

debug.Forces = [Fx;Fy;Fz];
debug.Moments = [L;M;N];

debug.quat_rates = [q0dot;q1dot;q2dot;q3dot];

debug.earth_velocities = XYZdot;

debug.L_be = L_be;
debug.L_eb = L_eb;

end