% Initialise matrices A_Lon,B_Lon,A_Lat,B_Lat.
% Input:
%   CG1 or CG2  - 0/1 -> CG position CG1 or CG2
%   v100 or 180 - 0/1 -> airsoeed 100kn or 180kn
% Output:
%   FlightData - Flight Data from LoadFlightDataPC9_CG2 or LoadFlightDataPC9_nominalCG1
%   X0 - init state vector
%   U0 - init control vector
%   A_Lon - matrix A for Linear Longitudinal Motion
%   B_Lon - matrix B for Linear Longitudinal Motion
%   A_Lat - matrix A for Linear Lateral-Directional Motion
%   B_Lat - matrix B for Linear Lateral-Directional Motion
%
function [FlightData,X0,U0,A_Lon,B_Lon,A_Lat,B_Lat] = PartCinit(CG1orCG2,v100or180)

    if (CG1orCG2 == 0)
        FlightData = aero3560_LoadFlightDataPC9_nominalCG1();%
        if (v100or180 == 0)
            % load initial conditions
            load('ICs_PC9_nominalCG1_100Kn_1000ft','-mat','X0','U0');
            % load longitudinal matrices
            load('Longitudinal_Matrices_PC9_nominalCG1_100Kn_1000ft','-mat','A_Lon','B_Lon');
        else
            % load initial conditions
            load('ICs_PC9_nominalCG1_180Kn_1000ft','-mat','X0','U0');
            % load longitudinal matrices
            load('Longitudinal_Matrices_PC9_nominalCG1_180Kn_1000ft','-mat','A_Lon','B_Lon');
        end
    else
        FlightData = aero3560_LoadFlightDataPC9_CG2();%
        if (v100or180 == 0)
            % load initial conditions
            load('ICs_PC9_CG2_100Kn_1000ft','-mat','X0','U0');
            % load longitudinal matrices
            load('Longitudinal_Matrices_PC9_CG2_100Kn_1000ft','-mat','A_Lon','B_Lon');
        else
            % load initial conditions
            load('ICs_PC9_CG2_180Kn_1000ft','-mat','X0','U0');
            % load longitudinal matrices
            load('Longitudinal_Matrices_PC9_CG2_180Kn_1000ft','-mat','A_Lon','B_Lon');
        end
    end

    V = AeroAngles(X0(1),X0(2),X0(3));
    flowproperties = FlowProperties(-X0(12),V,1);
    QS = flowproperties(2)*FlightData.Geo.S;

    % init lateral-directional matrices
    Yv  = QS*FlightData.Aero.Cyb/FlightData.Inertial.m/V;
    Yp  = QS*FlightData.Aero.Cyp*FlightData.Geo.b/FlightData.Inertial.m/V/2;
    Yr  = QS*FlightData.Aero.Cyr*FlightData.Geo.b/FlightData.Inertial.m/V/2;
    Lv  = QS*FlightData.Aero.Clb*FlightData.Geo.b/FlightData.Inertial.Ixx/V;
    Lp  = QS*FlightData.Aero.Clp*FlightData.Geo.b*FlightData.Geo.b/FlightData.Inertial.Ixx/V/2;
    Lr  = QS*FlightData.Aero.Clr*FlightData.Geo.b*FlightData.Geo.b/FlightData.Inertial.Ixx/V/2;
    Nv  = QS*FlightData.Aero.Cnb*FlightData.Geo.b/FlightData.Inertial.Izz/V;
    Np  = QS*FlightData.Aero.Cnp*FlightData.Geo.b*FlightData.Geo.b/FlightData.Inertial.Izz/V/2;
    Nr  = QS*FlightData.Aero.Cnr*FlightData.Geo.b*FlightData.Geo.b/FlightData.Inertial.Izz/V/2;
%     Yda = QS*FlightData.Aero.Cyda/FlightData.Inertial.m;
    Ydr = QS*FlightData.Aero.Cydr/FlightData.Inertial.m;
    Lda = QS*FlightData.Aero.Clda*FlightData.Geo.b/FlightData.Inertial.Ixx;
    Ldr = QS*FlightData.Aero.Cldr*FlightData.Geo.b/FlightData.Inertial.Ixx;
    Nda = QS*FlightData.Aero.Cnda*FlightData.Geo.b/FlightData.Inertial.Izz;
    Ndr = QS*FlightData.Aero.Cndr*FlightData.Geo.b/FlightData.Inertial.Izz;
    
    A1  = FlightData.Inertial.Ixz / FlightData.Inertial.Ixx;
    B1  = FlightData.Inertial.Ixz / FlightData.Inertial.Izz;
    AB  = 1 - A1 * B1;
    
    A_Lat = [ Yv            Yp            Yr-V            FlightData.Inertial.g*cos(X0(8)) 0;
              (Lv+A1*Nv)/AB (Lp+A1*Np)/AB (Lr+A1*Nr)/AB   0                                0;
              (Nv+B1*Lv)/AB (Np+B1*Lp)/AB (Nr+B1*Lr)/AB   0                                0;
              0             1             tan(X0(8))      0                                0;
              0             0             sec(X0(8))      0                                0];
    B_Lat = [ 0               Ydr;
              (Lda+A1*Nda)/AB (Ldr+A1*Ndr)/AB;
              (Nda+B1*Lda)/AB (Ndr+B1*Ldr)/AB;
              0               0;
              0               0];
end