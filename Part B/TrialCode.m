% Flight Mech - A3
% Trying to see if the trim function works 
clear;
clc;
close all;

% Load in the flight data
FlightData=aero3560_LoadFlightDataPC9_CG2;
V=100*0.5144;
h = 304.8;

% Checking the trim function
Trimmed = Trim(FlightData,V,h);
X0=Trimmed.X0;
U0=Trimmed.U0;
%% State Rates Check
X=X0;
U=U0;
Xdot = StateRates(X,FlightData,h,U);



