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
Trimmed = Trim(FlightData,V,h);



% inputs-------------------------------------------------------------------

delta_e = 
delta_t =
delta_a = 
delta_r = 


