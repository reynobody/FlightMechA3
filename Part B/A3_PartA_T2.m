% Flight Mech A3
clear;
clc;
close all;

% Change/load stuff for the correct flight condition
FlightData=aero3560_LoadFlightDataPC9_nominalCG1;
% Pick out the data that will be needed
beta = deg2rad(-3);
m = FlightData.Inertial.m;
W = FlightData.Inertial.m*FlightData.Inertial.g;
V = 100*0.5144;
rho = 1.225;
S = FlightData.Geo.S;
b = FlightData.Geo.b;

% Y force - CHANGE
CYb = FlightData.Aero.Cyb;
CYda = FlightData.Aero.Cyda;
CYdr = FlightData.Aero.Cydr;
CYr = FlightData.Aero.Cyr;
% N moment
Cnb = FlightData.Aero.Cnb;
Cnda = FlightData.Aero.Cnda;
Cndr = FlightData.Aero.Cndr;
Cnr = FlightData.Aero.Cnr;
% l moment
Clb = FlightData.Aero.Clb;
Clda = FlightData.Aero.Clda;
Cldr = FlightData.Aero.Cldr;
Clr = FlightData.Aero.Clr;

r = pi/15; %rad/s
r_hat = r*b/(2*V);

CL = W/(0.5*rho*V^2*S);

D = [Clda Cldr;
     Cnda Cndr];
E = -[Clr;Cnr]*r_hat;
F = D\E;
da = F(1);
dr = F(2);

phi = (-CYdr*dr-CYr*r_hat+2*m/(rho*V*S)*r)/CL;

nz = 1/cos(phi);
psi_dot = r*cos(phi);

% Part c
A  = [Clb Clda;
      Cnb Cnda];
B = -[Cldr;Cndr]*r_hat;
C = A\B;

% Trim settings for aileron only turn
da_trim = (Clb*Cnr-Cnb*Clr)/(Clda*Cnb*(1-Cnda/Clda*Clb/Cnb))*r_hat;
b_trim = (Cnda*Clr-Cnr-Clda)/Clda/Cnb/(1-Cnda/Clda*Clb/Cnb)*r_hat;


fprintf('a) CL=%f, nz=%f, psi_dot = %f, phi=%f, r=%f\n',CL,nz,rad2deg(psi_dot),rad2deg(phi),rad2deg(r));
fprintf('b)da=%f and dr=%f\n',rad2deg(da),rad2deg(dr));
fprintf('c)beta=%f, da=%f, da_trim%f, beta_trim=%f\n',rad2deg(C(1)),rad2deg(C(2)),rad2deg(da_trim),rad2deg(b_trim));

if Clb*Cnr-Cnb*Clr>0
    fprintf('d) STABLE\n');
elseif Clb*Cnr-Cnb*Clr<0
    fprintf('d) UNSTABLE\n');
else
    fprintf('d) NEUTRAL\n');
end
