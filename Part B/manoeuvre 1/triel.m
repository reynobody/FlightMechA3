% Flight Mech - A3
% Trying to see if the trim function works 
clear;
clc;
close all;

% Load in the flight data
FlightData=aero3560_LoadFlightDataPC9_nominalCG1;
V=100*0.5144;
h = 304.8;

% Checking the trim function
Trimmed = Trim(FlightData,V,h);
X0=Trimmed.X0;
U0=Trimmed.U0;
% %% State Rates Check
%  load('ICs_PC9_nominalCG1_100Kn_1000ft.mat');
% X=zeros(13,1);
% X(1:6) = X0(1:6);
% X(7:10) = e2q(rad2deg(X0(7:9)));
% X(11:13) = X0(10:12);

% X0 = [51.3569,    0.1000,    4.4368,    0.1000,    0.1000,    0.1000,    0.9935,    0.0451,    0.0943,    0.0451,    0.1000,    0.1000, -304.7000]';
% U0=[0.1929,    0.0958,    0.1000 ,   0.1000]';
X=X0;
U=U0;
[Xdot,debug] = StateRates(X,FlightData,h,U);
Xmat = X;
dt = 0.1;
U_mat = [];

time = 0:dt:150;

for i = 1:length(time)

    if time(i)<5
        U = U0;
    elseif time(i)<5.5
        U(2) = U0(2)+deg2rad(5);
    else
        U = U0;
    end


    Xnew = Integrate(X,FlightData,h,U,dt);
    Xmat = [Xmat,Xnew];
    U_mat = [U_mat,U];
    X = Xnew;
end

figure(1);
plot3(Xmat(11,:),Xmat(12,:),-Xmat(13,:));
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
grid minor;
axis tight;
title('Position');
saveas(gcf,'man1case1out2.pdf');

figure(2);
plot(time,rad2deg(U_mat(2:4,:)));
xlabel('Time [s]');
ylabel('Deflection [^\circ]');
grid on;
grid minor;
axis tight;
title('Controls');
saveas(gcf,'man1case1in.pdf');
