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
