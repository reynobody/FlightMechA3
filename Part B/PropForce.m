%% PROPULSIVE FORCES %%%
function [Thrust] = PropForce(FlightData, X, h, U)

Eta = FlightData.Prop.eta;
P_maxSL = FlightData.Prop.P_max;
[V,~,~]=AeroAngles(X);
dt_T = U(1);
density_SL = 1.225;     % kg/m^3
[rho,~]=FlowProperties(h,V);
sigma = rho/density_SL; % /Sigma is Density Ratio
Pmax= P_maxSL*(1.1324*sigma - 0.1324); % max power for the engine
Thrust = ((Eta*Pmax)/V)*dt_T; %% Thrust %%

end