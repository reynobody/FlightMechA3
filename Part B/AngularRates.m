% Function to estimate alpha_dot and beta_dot

function ret = AngularRates(X,Xdot)

% Extract body accelerations components
udot = Xdot(1);
vdot = Xdot(2);
wdot = Xdot(3);

% Extract body velocities components
u = X(1);
v = X(2);
w = X(3);

% Calculate V
V = sqrt(u^2+v^2+w^2);
Vdot = (u*udot+v*vdot+w*wdot)/V;

% Calculate total alpha dot (alpha rate of change)
alphadot = (u*wdot-w*udot)/(u^2+w^2);

% Calculate total beta dot (beta rate of change)
betadot = (vdot/V - v*Vdot/V^2)/sqrt(1-v^2/V^2);

% Form an output vector
ret.alphadot = alphadot;
ret.betadot = betadot;

end


% 
% if cg == 1
%     [ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();
% elseif cg == 2
%     [ FlightData ] = aero3560_LoadFlightDataPC9_CG2();
% end
% 
% u1 = ;
% rho = 1.225;
% Q = 0.5*rho*u1^2;
% 
% Zu = ;
% Za = ;
% Du = ;
% Da = ;
% Dq = ;
% Zdt = ;
% Zde = ;
% Ddt = ;
% Dde = ;
% 
% alphadot = Zu/u1*Du + Za/u1*Da + Dq;
% 
% Yb = Q*FlightData.Geo.S*FlightData.Aero.Cyb/FlightData.Inertial.m;
% Yp = Q*FlightData.Geo.S*FlightData.Geo.b*FlightData.Aero.Cyp/...
%     (2*FlightData.Inertial.m*u1);
% Yr = Q*FlightData.Geo.S*FlightData.Geo.b*FlightData.Aero.Cyr/...
%     (2*FlightData.Inertial.m*u1);
% theta1 = ;
% Db = ;
% Dp = ;
% Dr = ;
% Dphi = ;
% Dpsi = ;
% Ydr = Q*FlightData.Geo.S*FlightData.Geo.b*FlightData.Aero.Cydr;
% Dda = ;
% Ddr = ;
% 
% betadot = Yb/u1*Db + Yp/u1*Dp + (Yr/u1-1)*Dr + g*cos(theta1)/u1*Dphi + ...
%     Ydr/u1*Ddr;
% 
% ret.alphadot = alphadot;
% ret.betadot = betadot;