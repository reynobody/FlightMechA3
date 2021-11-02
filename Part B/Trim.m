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
[rho,Q] = FlowProperties(h,V);

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
dP = tol;

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
    
    
    % Converting to quaternions
    quaternions = e2q(euler);
    
    % Place the quaternions into X
    X(7:10) = quaternions';
    X_approx_dot = StateRates(X,FlightData,h,U);
    
    % Small pertubations
    
    % Angle of attack alpha
    Xalpha = X;
    Xalpha(1) = V*cos(X_approx(1)+dP);
    Xalpha(3) = V*sin(X_approx(1)+dP);
    
    % Finding the angular rate of alpha
    X_approx_dotA = StateRates(Xalpha,FlightData,h,U);
    
    % Finding the bits of the Jacobian matrix
    J(:,1) = (X_approx_dotA-X_approx_dot)/dP;
    
    % Do the same for throttle setting
    UT(1) = X_approx(2)+dP;
    UT(2) = X_approx(3);
    
    X_approx_dotT = StateRates(X,FlightData,h,UT);
    J(:,2) = (X_approx_dotT-X_approx_dot)/dP;
    
    % Elevator deflection
    Ude(1) = X_approx(2);
    Ude(2) = X_approx(3)+dP;
    X_approx_dotE = StateRates(X,FlightData,h,Ude);
    J(:,3) = (X_approx_dotE-X_approx_dot)/dP;
    
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
qs = e2q(e);
X0(7:10) = qs';
X0(13) = -h; % Up is down
U0 = [X_approx(2) X_approx(3) 0 0]';
Trimmed.X0 = X0;
Trimmed.U0 = U0;

end