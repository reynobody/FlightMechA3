% main
clear
clf 
clf reset
clc

condition = 1;
h = 1000*0.3048; % m

% initialisation-----------------------------------------------------------
[FlightData,IC,V] = Initialisation(condition);

% trim---------------------------------------------------------------------
h = IC.X0(end);
Trimmed = Trim(FlightData,V,h);

X0 = Trimmed.X0;
U0 = Trimmed.U0;

% inputs-------------------------------------------------------------------

ControlInputs = Controls(condition);

time_e = ControlInputs.X_de;
delta_e = ControlInputs.Y_de;

time_t = ControlInputs.X_dt;
delta_t = ControlInputs.Y_dt;

time_a = ControlInputs.X_da;
delta_a = ControlInputs.Y_da; 

time_r = ControlInputs.X_dr;
delta_r = ControlInputs.Y_dr; 

U0_GUI = [delta_e'; delta_t'; delta_a'; delta_r'];

% Not sure if the stopping and stepping time are fixed with ControlGUI 
% Probs not, ill still leave this here tho we can delete later
timestop = ControlInputs.SimTime;
timestep = ControlInputs.TimeStep;

count = 0;
count_c = 0;
for time=1:timestep:timestop
    
    % Update counter
    count = count+1;
    
    % Start control only at t = 5
    if time >= 5
        
        % Update control counter
        count_c = count_c + 1;
        
        % Get control input from the GUI
        U0 = U0_GUI(:,count_c);
        
        % Integrate the trajectory at each time step
        X_new = Integrate(X0,FlightData,h,U0,dt);
        
    else
        % Integrate without control input
        X_new = Integrate(data,FC,dt);
    end
    
    % Update states and rates
    X0 = X_new;

%     % IM NOT SURE ABOUT THESE FOLLOWING LINES, WHAT THEY MEAN
%     % Form the state matrix for each time step
%     X_mat(:,count) = X0; 
%     
%     % Form the control matrix for each time step
%     control(:,count)=FC.U0;
