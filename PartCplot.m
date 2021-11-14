% Plot time-history of each of the state variables in response to 
%   impulsive control inputs.
% The state variables to look at are (u, α, q, θ, ze) for the longitudinal 
%   dynamics and (β, p, r, ϕ, ψ) for the lateral-directional dynamics.
% Input:
%       t - time vector
%       V - true airspeed
%       U0  - init control vector
%       SysLon - state-space model for Longitudinal Motion
%       SysLat - state-space model for Lateral Motion
%       name - simulation name (for figure name)
%
function PartCplot(t,V,U0,SysLon,SysLat,name)
    % control vectors
    udT = @(t) ones(length(t),1)*U0(1);
    ude = @(t) d2r(5)*(t>=1.0).*(t<=1.5);
    % responce simulation (elevator impulse of magnitude 5deg and lasting 0.5s)
    sysLon = tf(SysLon);
    figure('units','normalized','outerposition',[0 0 1 1],'Name',name)
    x1 = lsim(sysLon,'-r', [udT(t) ude(t)'], t);
    subplot(5,1,1)
    plot(t,r2d(ude(t))); grid on
    xlabel('Time, s'); ylabel('\delta_e, deg'); ylim([-1 6])
    title('Input (elevator impulse)')
    subplot(5,1,2)
    plot(t,x1(:,1)); grid on
    xlabel('Time, s'); ylabel('u, m/s')
    title('Outputs')
    subplot(5,1,3)
    plot(t,x1(:,2),t,r2d(x1(:,4))); grid on
    % plot(t,r2d(x1(:,2)/V),t,r2d(x1(:,4))); grid on
    xlabel('Time, s'); legend('\alpha, deg','\theta, deg')
    subplot(5,1,4)
    plot(t,r2d(x1(:,3))); grid on
    xlabel('Time, s'); ylabel('q, deg/s')
    subplot(5,1,5)
    plot(t,(x1(:,5))); grid on
    xlabel('Time, s'); ylabel('z_e, m')
    % control vectors
    uda = @(t) d2r(5)*(t>=1.0).*(t<=1.5);
    udr = @(t) zeros(length(t),1);
    % responce simulation (aileron impulse of magnitude 5deg and lasting 0.5s)
    sysLat = tf(SysLat);
    figure('units','normalized','outerposition',[0 0 1 1],'Name',name)
    x2 = lsim(sysLat,'-r', [uda(t)' udr(t)], t);
    subplot(3,1,1)
    plot(t,r2d(uda(t)),t,r2d(udr(t))); grid on
    xlabel('Time, s'); legend('\delta_a, deg', '\delta_r, deg'); ylim([-1 6])
    title('Input (aileron impulse)')
    subplot(3,1,2)
    plot(t,r2d(x2(:,1)/V),t,r2d(x2(:,4)),t,r2d(x2(:,5))); grid on
    xlabel('Time, s'); legend('\beta, deg', '\phi, deg', '\psi, deg')
    title('Outputs')
    subplot(3,1,3)
    plot(t,r2d(x2(:,2)),t,r2d(x2(:,3))); grid on
    xlabel('Time, s'); legend('p, deg/s', 'r, deg/s')
    % control vectors
    uda = @(t) zeros(length(t),1);
    udr = @(t) d2r(5)*(t>=1.0).*(t<=1.5);
    % responce simulation (rudder impulse of magnitude 5deg and lasting 0.5s)
    figure('units','normalized','outerposition',[0 0 1 1],'Name',name)
    x3 = lsim(sysLat,'-r', [uda(t) udr(t)'], t);
    subplot(3,1,1)
    plot(t,r2d(uda(t)),t,r2d(udr(t))); grid on
    xlabel('Time, s'); legend('\delta_a, deg', '\delta_r, deg'); ylim([-1 6])
    title('Input (rudder impulse)')
    subplot(3,1,2)
    plot(t,r2d(x3(:,1)/V),t,r2d(x3(:,4)),t,r2d(x3(:,5))); grid on
    xlabel('Time, s'); legend('\beta, deg', '\phi, deg', '\psi, deg')
    title('Outputs')
    subplot(3,1,3)
    plot(t,r2d(x3(:,2)),t,r2d(x3(:,3))); grid on
    xlabel('Time, s'); legend('p, deg/s', 'r, deg/s')
end