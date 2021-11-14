%% Body Forces %%

function [BForces, l, m, n,debug] = BodyForces(FlightData, X, AngularRate ,h, U)

[V, alpha, beta] = AeroAngles(X);

p = X(4);     q = X(5);       r = X(6);
de = U(2);     da = U(3);     dr = U(4);

% Force and moment coefficients

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

% Friendly geometry
S = FlightData.Geo.S;
c = FlightData.Geo.c;
b = FlightData.Geo.b;

% Finding the value of phi which we'll need later
quaternions=X(7:10);
eulerAngles=deg2rad(q2e(quaternions));
phi = eulerAngles(1);

[L, D,debug] = WindForces(FlightData,X,AngularRate,h,U);
[rho,Q]=FlowProperties(h,V);

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

Y = 0.5*rho*S*CY*V^2;           % Side Force
l = 0.5*rho*S*b*Cl*V^2;        % Moment about L.
m = 0.5*rho*S*c*CM*V^2;        % Moment about M.
n = 0.5*rho*S*b*CN*V^2;        % Moment about N.

Cba = DCM(e2q(rad2deg([0 alpha -beta])));
BForces = Cba*[-D;0;-L];
BForces(2)=Y;

% Debugs
debug.V = V;
debug.alpha = alpha;
debug.beta = beta;
debug.alpha_d = ad;
debug.beta_d = bd;
debug.alpha_dhat = ad_hat;
debug.beta_dhat = bd_hat;
debug.phat = p_hat;
debug.rhat = r_hat;
debug.qhat = q_hat;
debug.rho = rho;
debug.Q = Q;
debug.Cy = CY;
debug.Cl = Cl;
debug.Cm = CM;
debug.Cn = CN;
debug.Mx = l;
debug.My = m;
debug.Mz = n;

end