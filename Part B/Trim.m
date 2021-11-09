% Aero3560 Assignment 3
% Trim function
function Trimmed = Trim(FlightData,V,h)


% Load in the data that we'll need
S = FlightData.Geo.S;       
c = FlightData.Geo.c;       
b = FlightData.Geo.b;       

% Inertial Data
m = FlightData.Inertial.m;	 
g = FlightData.Inertial.g;   
W = m*g;                     

% Lift and Drag coefficients
CLo = FlightData.Aero.CLo;  
CLa = FlightData.Aero.CLa;  

% Set up the State and Control vectors
X = zeros(13,1);    % It's supposed to be a nice vertical vector
X(13) = -h; % Up is NEGATIVE
U = zeros(4,1); % Yer control vector
UT = U;       % Thrust
Ude = U;       % Elevator

% Dynamic Pressure (Q) from the flow properties
[~,Q] = FlowProperties(h,V);

% Finding the initial estimates for everythig
% For steady level flight, lift = weight
CL = W/(Q*S);
alpha = (CL-CLo)/CLa; % This is approximately true for the linear bit of the lift curve slope
dT = 0.5; % Nice number for estimating
de = 0; % Assuming nothing strange yet

% Creating the random state vector that was made by the above
% approximations
X_approx = [alpha; dT; de];

% Generate a small tolerance
tol = 10^(-8);

% Start with a large value for error
error = 1;

% Set the pertubations to be very small
dx = tol;

while error > tol
    
    % Velocity components
    X(1) = V*cos(X_approx(1));
    X(3) = V*sin(X_approx(1));
    
    % Thrust and elevator settings
    U(1) = X_approx(2);
    U(2) = X_approx(3);
    
    % Finding the desired equilibrium condition
    phi = 0;
    theta = X_approx(1);
    psi = 0;
    euler = [phi theta psi];
    euler = rad2deg(euler); % Need to convert because I made the e2q and q2e functions to use degrees
    
    % Converting to quaternions
    quaternions = e2q(euler);
    
    % Place the quaternions into X
    X(7:10) = quaternions';
    X_approx_dot = StateRates(X,FlightData,h,U);
    
    %% Small pertubations
    
    % Angle of attack alpha
    Xa_p = X;
    Xa_p(1) = V*cos(X_approx(1)+dx);
    Xa_p(3) = V*sin(X_approx(1)+dx);

        Xa_m = X;
    Xa_m(1) = V*cos(X_approx(1)-dx);
    Xa_m(3) = V*sin(X_approx(1)-dx);
    
    % Finding the angular rate of alpha
    fXa_p = StateRates(Xa_p,FlightData,h,U);
        fXa_m = StateRates(Xa_m,FlightData,h,U);
    
    % Finding the bits of the Jacobian matrix
    J(:,1) = (fXa_p-fXa_m)/(2*dx);
    
    %% Throttle
    % Do the same for throttle setting
    UT_p = U;
    UT_p(1) = X_approx(2)+dx;
    UT_m = U;
    UT_m(1) = X_approx(2)-dx;
    
    fUT_p = StateRates(X,FlightData,h,UT_p);
    fUT_m = StateRates(X,FlightData,h,UT_m);
    J(:,2) = (fUT_p-fUT_m)/(2*dx);
    

   %% Elevator
    % Elevator deflection
    Ue_p = U;
    Ue_p(2) = X_approx(3)+dx;
    Ue_m = U;
    Ue_m(2) = X_approx(3)-dx;
    
    fUe_p = StateRates(X,FlightData,h,Ue_p);
    fUe_m = StateRates(X,FlightData,h,Ue_m);
    J(:,3) = (fUe_p-fUe_m)/(2*dx);
    
    %% Jacobian SORTING
    % The Jacobian Matrix becomes
    Jacobian = [J(1,1) J(1,2) J(1,3);
                J(3,1) J(3,2) J(3,3);
                J(5,1) J(5,2) J(5,3)];
    
    % Find a new estimate of the state variables
    X_approx_new = X_approx-Jacobian\[X_approx_dot(1); X_approx_dot(3); X_approx_dot(5)];
    error = sum(abs(X_approx_new-X_approx));
    
    X_approx = X_approx_new;
    
end
% Finding the new components for the trimmed state
X0 = zeros(13,1);
X0(1:3) = [V*cos(X_approx(1));0;V*sin(X_approx(1))];

e = [0 X_approx(1) 0];
e = rad2deg(e);
qs = e2q(e);
X0(7:10) = qs';
X0(13) = -h; % Up is down
U0 = [X_approx(2) X_approx(3) 0 0]';
Trimmed.X0 = X0;
Trimmed.U0 = U0;

end