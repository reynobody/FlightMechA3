%% WIND FORCES%%


function [L,D] = WindForces(FlightData,X,AngularRate,h,U)

% Pulling the Aircraft data from the original files
CLa = FlightData.Aero.CLa;
CLq = FlightData.Aero.CLq;
CLad = FlightData.Aero.CLad;
CLde = FlightData.Aero.CLde;
CLo = FlightData.Aero.CLo;
Cdo = FlightData.Aero.Cdo;
k = FlightData.Aero.k;
S = FlightData.Geo.S;
c = FlightData.Geo.c;

[V,alpha,~]=AeroAngles(X);
q = X(5); d_e = U(2);

[Rho,~] = FlowProperties(h,V);

ad=AngularRate(1);

q_cap = (q*c)/(2*V);
CL = CLo + CLa*alpha + CLad*ad + CLq*q_cap + CLde*d_e;% Lift Coefficient
CD = Cdo + k*(CL^2); % Coefficient of Drag
L = CL*(0.5*Rho*S*V^2); % The Lift
D = CD*(0.5*Rho*S*V^2); % The Drag

end