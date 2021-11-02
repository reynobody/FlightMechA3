%% Body Forces %%

function [BForces, m, n, l] = BodyForces(FlightData, X, AngularRate ,h, U)

[V, alpha, beta] = AeroAngles(X);
p = X(4);     q = X(5);       r = X(6);
de = U(2);     da = U(3);     dr = U(4);

%Redefining these , however we can erase them and still continue to use as
%the functions are already defiend at various places or we can create a
%Data file creating these and use the data file for all the necessary functions.

% Side Force
Cyb = FlightData.Aero.Cyb;
Cybd = FlightData.Aero.Cybd;
Cyp = FlightData.Aero.Cyp;
Cyr = FlightData.Aero.Cyr;
Cyda = FlightData.Aero.Cyda;
Cydr = FlightData.Aero.Cydr;

% M Moment
Cm0 = FlightData.Aero.Cmo;
Cma = FlightData.Aero.Cma;
Cmq = FlightData.Aero.Cmq;
Cmad = FlightData.Aero.Cmad;
Cmde = FlightData.Aero.Cmde;

% N Moment
Cnb = FlightData.Aero.Cnb;
Cnbd = FlightData.Aero.Cnbd;
Cnp = FlightData.Aero.Cnp;
Cnr = FlightData.Aero.Cnr;
Cnda = FlightData.Aero.Cnda;
Cndr = FlightData.Aero.Cndr;

% L Moment
Clb = FlightData.Aero.Clb;
Clbd = FlightData.Aero.Clbd;
Clp = FlightData.Aero.Clp;
Clr = FlightData.Aero.Clr;
Clda = FlightData.Aero.Clda;
Cldr = FlightData.Aero.Cldr;

S = FlightData.Geo.S;
c = FlightData.Geo.c;
b = FlightData.Geo.b;

quaternions=X(7:10);
eulerAngles=q2e(quaternions);
phi = eulerAngles(1);

[L, D] = WindForces(FlightData,X,AngularRate,h,U);
[rho,~]=FlowProperties(h,V);

p_hat = (p*b)/(2*V);      q_hat = (q*c)/(2*V);
r_hat = (r*b)/(2*V);

ad=AngularRate(1);
bd=AngularRate(2);
ad_hat=ad*c/(2*V);
bd_hat = bd*b/(2*V);

%Input the formulae to identify the moments and force vectors.
CY = Cyb*beta + Cybd*bd_hat + Cyp*p_hat + Cyr*r_hat +Cyda*da + Cydr*dr;
Cl = Clb*beta + Clbd*bd_hat + Clp*p_hat + Clr*r_hat +Clda*da + Cldr*dr;
CM = Cm0 + Cma*alpha + Cmq*q_hat + Cmad*ad_hat + Cmde*de;
CN = Cnb*beta + Cnbd*bd_hat + Cnp*p_hat + Cnr*r_hat +Cnda*da + Cndr*dr;

Y = 0.5*rho*S*CY*V^2+L*deg2rad(phi);           % Side Force
l = 0.5*rho*S*b*Cl*V^2;        % Moment about L.
m = 0.5*rho*S*c*CM*V^2;        % Moment about M.
n = 0.5*rho*S*b*CN*V^2;        % Moment about N.

Cba = Cy(rad2deg(alpha))*Cz(rad2deg(-beta));
BForces = Cba*[-D;0;-L];
BForces(2)=Y;

end