%% Part C – Handling Qualities
%
close all
clearvars
clc
format compact 

disp('Part C')
disp(['Date and Time: ', num2str(datestr(now))])
disp('-----------------------------------------')
% load data
CG1orCG2 = input('is Load Flight Data PC9 CG1 (Y/N)? ','s');   
if isempty(CG1orCG2)
    CG1orCG2 = '0';
end
if ((CG1orCG2 == 'Y') || (CG1orCG2 == 'y'))
    CG1orCG2 = 0;
    disp('---> Load Flight Data PC9 is CG1')
    nameCG = 'PC9 CG1';
else 
    if ((CG1orCG2 == 'N') || (CG1orCG2 == 'n'))
        CG1orCG2 = 1;
        disp('---> Load Flight Data PC9 is CG2')
    nameCG = 'PC9 CG2';        
    else
        disp('---> done: EXIT')
        return;
    end
end  
v100or180 = input('is ICs 100Kn (Y/N)? ','s');   
if isempty(v100or180)
    v100or180 = '0';
end           
if ((v100or180 == 'Y') || (v100or180 == 'y'))
    v100or180 = 0;
    disp('---> Load ICs is 100Kn 1000ft')
    name = nameCG + " 100Kn 1000ft";
else 
    if ((v100or180 == 'N') || (v100or180 == 'n'))
        v100or180 = 1;
        disp('---> Load ICs is 180Kn 1000ft')
        name = nameCG + " 180Kn 1000ft";
    else
        disp('---> done: EXIT')
        return;
    end
end
[FlightData,X0,U0,A_Lon,B_Lon,A_Lat,B_Lat] = PartCinit(CG1orCG2,v100or180);
% eigenvalues of the A matrices
V = AeroAngles(X0(1),X0(2),X0(3));
disp(['---> Longitudinal modes ' nameCG ' at ' num2str(V) ' m/s'])
% [num,den] = ss2tf(A_Lon,B_Lon,eye(5),zeros(5,2),2);
SysLon = ss(A_Lon,B_Lon,eye(5),zeros(5,2));
% sys = tf(Sys);
% The eigenvalues of the A matrices
disp('eigenvalues:')
disp(eig(SysLon))
% system pole indication 
disp('system pole indication:') 
damp(SysLon)
 SysLat = ss(A_Lat,B_Lat,eye(5),zeros(5,2));
timeStart = 0;
timeStop = 10;
timeStep = 0.01;
t = linspace (timeStart,timeStop,(timeStop-timeStart)/timeStep+1);
% plot response of the aircraft to elevator, aileron, and rudder impulse inputs
PartCplot(t,V,U0,SysLon,SysLat,name);
disp('=========================================')