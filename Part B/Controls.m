% Control function for FM assignment 3 
% Outputs control functions and timestep information
% - X_dt
% - Y_dt
% - X_de
% - Y_de
% - X_da
% - Y_da
% - X_dr
% - Y_dr
% - X_df
% - Y_df
% - SimTime (simulated amount of time in seconds)
% - TimeStep (time step in seconds)
% - freq (frequency of outputs)

function output = Controls(casenum)

if casenum == 1
    filename = 'Control_Data1';
elseif casenum == 2
    filename = 'Control_Data2';
elseif casenum == 3
    filename = 'Control_Data3';
elseif casenum == 4
    filename = 'Control_Data4';
elseif casenum == 5
    filename = 'Control_Data5';
elseif casenum == 6
    filename = 'Control_Data6';
elseif casenum == 7
    filename = 'Control_Data7';
elseif casenum == 8
    filename = 'Control_Data8';
end

load_data = load(filename);
output = load_data;

end


