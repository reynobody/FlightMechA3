% main
clear
clc
close all
clf
clf reset

condition = 1;

% initialisation-----------------------------------------------------------
[FlightData,IC,V] = Initialisation(condition);

% trim---------------------------------------------------------------------
h = IC.X0(end);
Trimmed = Trim(FlightData,V,h);

X0 = Trimmed.X0;
U0 = Trimmed.U0;

% inputs-------------------------------------------------------------------

flightcase = 1; % NOT CONDITION: this is questions 1-8
ControlInputs = Controls(flightcase);

U_GUI = ControlInputs.U_linear(1:4,:);

% Not sure if the stopping and stepping time are fixed with ControlGUI
% Probs not, ill still leave this here tho we can delete later
time = ControlInputs.T_linear;
dt = time(2) - time(1);

count = 0;
count_c = 0;

for i = 1:length(time)
    
    h = -X0(13);
        
     % Get control input from the GUI
     % This really depends if you're using control gui outputs in degs or rads
     U = U0 + deg2rad(U_GUI(:,i));
        
     % Integrate the trajectory at each time step
     X_new = Integrate(X0,FlightData,h,U,dt);
        
    
    % Update states and rates
    X0 = X_new;
    
    % Form the state matrix for each time step
    X_mat(:,i) = X0;
    
    % Form the control matrix for each time step
    control(:,i) = U;
    
end

PlotData(time, X_mat, control);

