% Function takes input of which out of 4 cases is being simulated
% Function returns:
% - flightdata of current flight condition case
% - initial conditions
% - flightspeed

function [FlightData,IC,V] = Initialisation(condition)

if condition == 1
    V = 100*0.5144; % Flight speed in meters per second
    IC = load('ICs_PC9_nominalCG1_100Kn_1000ft.mat'); % Load initial conditions of CG=1, 100 KTS
    FlightData = aero3560_LoadFlightDataPC9_nominalCG1(); % Load flight data of CG=1
elseif condition == 2
    V = 180*0.5144; % Flight speed in meters per second
    IC = load('ICs_PC9_nominalCG1_180Kn_1000ft.mat'); % Load initial conditions of CG=1, 180 KTS
    FlightData = aero3560_LoadFlightDataPC9_nominalCG1(); % Load flight data of CG=1
elseif condition == 3
    V = 100*0.5144;
    IC = load('ICs_PC9_CG2_100Kn_1000ft.mat'); % Load initial conditions of CG=2, 100 KTS
    FlightData = aero3560_LoadFlightDataPC9_CG2(); % Load flight data of CG=1
elseif condition == 4
    V = 180*0.5144;
    IC = load('ICs_PC9_CG2_180Kn_1000ft.mat'); % Load initial conditions of CG=2, 180 KTS
    FlightData = aero3560_LoadFlightDataPC9_CG2(); % Load flight data of CG=1
end

end
