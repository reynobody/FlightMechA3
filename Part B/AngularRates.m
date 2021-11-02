% Function to estimate alpha_dot and beta_dot

function ret = AngularRates(Xdot)

% Body rates rates
udot = Xdot(1);
vdot = Xdot(2);
wdot = Xdot(3);

% body rates
u = Xdot(11);
v = Xdot(12);
w = Xdot(13);

% Calculate V
V = sqrt(u^2+v^2+w^2);
Vdot = (u*udot+v*vdot+w*wdot)/V;

% Calculate total alpha dot (alpha rate of change)
alphadot = (u*wdot-w*udot)/(u^2+w^2);

% Calculate total beta dot (beta rate of change)
betadot = (vdot/V - v*Vdot/V^2)/sqrt(1-v^2/V^2);

ret=[alphadot,betadot];
end

