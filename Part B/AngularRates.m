% Function to estimate alpha_dot and beta_dot

function ret = AngularRates(X,Xdot)

% Body rates rates
udot = Xdot(1);
vdot = Xdot(2);
wdot = Xdot(3);

% body rates
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

% Output to ret
ret.alphadot = alphadot;
ret.betadot = betadot;

end
