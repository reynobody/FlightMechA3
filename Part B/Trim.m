% Flight Mech Assignment 3
% TRIM FUNCTION

% Determine the controls and states for steady level flight

function  Trimmed = Trim(FlightData,V,h)
% Load in the Flight Data that we'll need 
S = FlightData.Geo.S;
g=FlightData.Inertial.g;           
m=FlightData.Inertial.m; 
W = m*g;
CLa = FlightData.Aero.CLa;
Clo= FlightData.Aero.CLo;

[~,Q]=FlowProperties(h,V);


% Set up the general X and U vectors
X = zeros(13,1);
U = zeros(4,1);

% Start by guessing alpha0, dT0 and de0
% Angle of Attack - alpha0
CL = W/(Q*S);
alpha0 = (CL-Clo)/CLa;

% Throttle - dT0 (gotta be between 0 and 1)
dT0 = 0.5; % From lecture slides

% Elevator - in radians
de0 = 0; % Nice friendly number


% Errors and tolerances
tol=10^(-8);
error = 1;

X_approx = [alpha0, dT0, de0]';

% Choose the small perturbance
dx = tol;

while error>tol

% Step 1
% Evaluate the expected terms
X(1) = V*cos(X_approx(1));
X(3) = V*sin(X_approx(1));

U(1) = dT0;
U(2) = de0;


% Step 2
% Assuming for steady level flight
phi = 0;
psi = 0;
theta = X_approx(1);

quaternions = e2q([phi, theta, psi]);
X(7:10)= quaternions;
X_dot = StateRates(X,FlightData,h,U);
%% Angle of Attack
Xa_p = X;
Xa_p(1) = V*cos(X(1)+dx);
Xa_p(1) = V*sin(X(1)+dx);

Xa_m = X;
Xa_m(1) = V*cos(X(1)-dx);
Xa_m(1) = V*sin(X(1)-dx);

% State Rates
fXa_m = StateRates(Xa_m,FlightData,h,U);
fXa_p = StateRates(Xa_p,FlightData,h,U);

J_temp(:,1) = (fXa_p-fXa_m)/(2*dx);

%% Throttle
UT_p = U;
UT_p(1)=U(1)+dx;
UT_m = U;
UT_m(1)=U(1)-dx;

fXT_m = StateRates(X,FlightData,h,UT_m);
fXT_p = StateRates(X,FlightData,h,UT_p);

J_temp(:,2) = (fXT_p-fXT_m)/(2*dx);

%% Elevator
Ue_p = U;
Ue_p(2)=U(2)+dx;
Ue_m = U;
Ue_m(2)=U(2)-dx;

feT_m = StateRates(X,FlightData,h,Ue_m);
feT_p = StateRates(X,FlightData,h,Ue_p);

J_temp(:,3) = (fXe_p-fXe_m)/(2*dx);

%% Jacobian Matrix
% Rearrange for the terms of interest

J = [J_temp(1,:); J_temp(3,:); J_temp(5,:)];
fX = [X_dot(1),X_dot(3),X_dot(5)]';
X_approx_new = X_approx-J\fX;
error = sum(abs(X_approx_new-X_approx));
X_approx = X_approx_new;
end

% Find the trimmed state
X0 = zeros(13,1);
x0(1:3)=[V*cos(X_approx(1)), 0, V*sin(X_approx(1))]';

euler = [0, X_approx(1), 0]';
quats = e2q(euler);
U0(7:10)=quats;
X0(13) = -h;
U0 = [X_approx(2), X_approx(3) 0, 0]';

Trimmed.X0 = X0;
Trimmed.U0 = U0;

end
