% AERO3560 ASS3 Taask 1

clear
clc


[ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();

% draw from data-----------------------------------------------------------

% Aerodynamic Data (Reference CG: 22 % mac)
alpha_0 = FlightData.Aero.alpha_o; 
% Drag Coefficients
Cd_0 = FlightData.Aero.Cdo;
k = FlightData.Aero.k;
% Lift Coefficients
CL_a = FlightData.Aero.CLa;
CL_q = FlightData.Aero.CLq;
CL_ad = FlightData.Aero.CLad;
CL_de = FlightData.Aero.CLde;
CL_0  = -CL_a*alpha_0;
% Side Force Coefficients
Cy_b = FlightData.Aero.Cyb;
Cy_bd = FlightData.Aero.Cybd;
Cy_p = FlightData.Aero.Cyp;
Cy_r = FlightData.Aero.Cyr;
Cy_da = FlightData.Aero.Cyda;
Cy_dr = FlightData.Aero.Cydr;
% M Moment Coefficients
Cm_0 = FlightData.Aero.Cmo;
Cm_a = FlightData.Aero.Cma;
Cm_q = FlightData.Aero.Cmq;
Cm_ad = FlightData.Aero.Cmad;
Cm_de = FlightData.Aero.Cmde;
% N Moment Coefficients
Cn_b = FlightData.Aero.Cnb;
Cn_bd = FlightData.Aero.Cnbd;
Cn_p = FlightData.Aero.Cnp;
Cn_r = FlightData.Aero.Cnr;
Cn_da = FlightData.Aero.Cnda;
Cn_dr = FlightData.Aero.Cndr;
% L Moment Coefficients
Cl_b = FlightData.Aero.Clb;
Cl_bd = FlightData.Aero.Clbd;
Cl_p = FlightData.Aero.Clp;
Cl_r = FlightData.Aero.Clr;
Cl_da = FlightData.Aero.Clda; 
Cl_dr = FlightData.Aero.Cldr;

Ixx = FlightData.Inertial.Ixx;
Iyy = FlightData.Inertial.Iyy;
Izz = FlightData.Inertial.Izz;
Ixz = FlightData.Inertial.Ixz;

% given/assumed values-----------------------------------------------------
V = 100/1.944;          % m/s
m = 2087;               % Kg
g = 9.81;
rho = 1.225;
S = 16.29;
beta = deg2rad(-7);
Q = 0.5*rho*V^2;
b = FlightData.Geo.b;

% needed for stuff---------------------------------------------------------

Y_b = Q*S*Cy_b/m;
Y_p = Q*S*b*Cy_p/(2*m*V);
Y_r = Q*S*b*Cy_r/(2*m*V);
Y_da = Q*S*Cy_da/m;
N_da = Q*S*b*Cn_da/Izz;
L_da = Q*S*b*Cl_da/Ixx;
N_b = Q*S*b*Cn_b/Izz;
N_p = Q*S*b^2*Cn_p/(2*Izz*V);
N_r = Q*S*b^2*Cn_r/(2*Izz*V);
Y_dr = Q*S*Cy_dr/m;
N_dr = Q*S*b*Cn_dr/Izz;
L_dr = Q*S*b*Cl_dr/Ixx;
L_b = Q*S*b*Cl_b/Ixx;
L_p = Q*S*b^2*Cl_p/(2*Ixx*V);
L_r = Q*S*b^2*Cl_r/(2*Ixx*V);


%==========================================================================

% Task 1a------------------------------------------------------------------

% lift coefficient
CL = (m*g)/(0.5*rho*V^2*S);

% drag coefficient
CD = Cd_0 + k*CL^2;
D = CD*0.5*rho*V^2 *S;


% formaulate the matrices for question 1

% ASSUMPTIONS
% steady flight hence, q, a and ad terms are assumed zero
% assumed stable flight so Cm is assumed zero

% moment coefficient and lift coefficients
M1 = [-Cm_0;CL-CL_0];
M2 = [Cm_a, Cm_de;CL_a,CL_de];

% solve
Ans1 = inv(M2)*M1;

% display
alpha = rad2deg(Ans1(1))
Elevator = rad2deg(Ans1(2))




% TASK 1b------------------------------------------------------------------

% formulate the matrices for question 2 (lecture 7B page 4)
M3 = [CL,0,Cy_dr;0,Cl_da,Cl_dr;0,Cn_da,Cn_dr];
M4 = [-Cy_b*beta;-Cl_b*beta;-Cn_b*beta];

% solve
Ans2 = inv(M3)*M4;

% yaw, ailer, rudder
phi = rad2deg(Ans2(1))
Aileron = rad2deg(Ans2(2))
Rudder = rad2deg(Ans2(3))


% TASK 2c------------------------------------------------------------------
% Airpath (V and LVLH)
% roll
phi = deg2rad(phi); % Converts the input from degrees to radians

Cx = [1 0          0;          
     0 cos(phi)   sin(phi);
     0 -sin(phi)  cos(phi)];
   

% climb angle
theta = -7;
 
theta = deg2rad(theta); % Converts the input from degrees to radians

Cy = [cos(theta)   0  -sin(theta);          
     0            1   0;
     sin(theta)   0   cos(theta)];

% angle of V from true north
psi = -25;

psi = deg2rad(psi); % Converts the input from degrees to radians

Cz = [cos(psi)  sin(psi) 0;          
     -sin(psi) cos(psi) 0;
     0        0        1 ];

 % V to LVLH
 Cva = Cx*Cy*Cz;
 
% body--------------------------------------------------------------------
% Aerodynamic angles (V and body)

% roll
phi = 0; % Converts the input from degrees to radians

Cx = [1 0          0;          
     0 cos(phi)   sin(phi);
     0 -sin(phi)  cos(phi)];

% climb angle
theta = alpha;
 
theta = deg2rad(theta); % Converts the input from degrees to radians

Cy = [cos(theta)   0  -sin(theta);          
     0            1   0;
     sin(theta)   0   cos(theta)];
 
% angle of V from true north
psi = beta;

psi = deg2rad(psi); % Converts the input from degrees to radians

Cz = [cos(psi)  sin(psi) 0;          
     -sin(psi) cos(psi) 0;
     0        0        1 ];
 
% V to body
Cba = Cx*Cy*Cz;

Cvb = Cva*Cba'

