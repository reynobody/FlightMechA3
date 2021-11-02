% main
clear
clf 
clf reset
clc

condition = 1;

% initialisation-----------------------------------------------------------
[FlightData,IC,V] = Initialisation(condition);

% trim---------------------------------------------------------------------
h = IC.X0(end);
Trimmed = Trim(FlightData,V,h);

X0 = Trimmed.X0;
U0 = Trimmed.U0;

% inputs-------------------------------------------------------------------

ControlInputs = Controls(condition);

U_GUI = ControlInputs.U_linear(1:4,:);

% Not sure if the stopping and stepping time are fixed with ControlGUI 
% Probs not, ill still leave this here tho we can delete later
time = ControlInputs.T_filter;
dt = time(2) - time(1);

count = 0;
count_c = 0;

for i = time
    
    % Update counter
    count = count+1;
    
    % Start control only at t = 5
    if  i >= 5
        
        % Update control counter
        count_c = count_c + 1;
        
        % Get control input from the GUI
        U = U_GUI(:,count_c);
        
        % Integrate the trajectory at each time step
        X_new = Integrate(X0,FlightData,h,U,dt);
        
    else
        % Integrate without control input
        X_new = Integrate(X0,FlightData,h,U0,dt);
        U = U0;
    end
    
    % Update states and rates
    X0 = X_new;

    % IM NOT SURE ABOUT THESE FOLLOWING LINES, WHAT THEY MEAN
    % Form the state matrix for each time step
    X_mat(:,count) = X0; 
     
    % Form the control matrix for each time step
    control(:,count)=U;
end

PlotData(time, X_mat, control);

