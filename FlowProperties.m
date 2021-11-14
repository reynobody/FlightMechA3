%% Modified Flow proerties

% Calculate the air density and dynamic pressure.
%   Additional flow properties: Temperature, Pressure, Mach number.
% Input:      h - Altitude (m or ft) {1ft = 0.3048m}
%             V - Airspeed (m/s or knot) {1kn = 0.514m/s}
%            SI - input data units = 1 or 0 -> metric units OR english units
% Output: fp(1) = rho - Air Density kg/m^3
%         fp(2) = q - Dynamic Pressure N/m^2 (Pa)
%         fp(3) = T - Temperature (°C)
%         fp(4) = P - Pressure N/m^2 (Pa)
%         fp(5) = M - Mach Number
%         fp(6) = sigma - Pressure Ratio
% 
function flowproperties = FlowProperties(h, V, SI)
    
    if SI
    % U.S. Standard atmosphere in metric units    
    hT = [0.0 500.00 1000.0 1500.0 2000.0 2500.00 3000.00 3500.00 4000.00 4500.00 5000.00;%Altitude (m)
       1.2250 1.1673 1.1116 1.0581 1.0065 0.95686 0.90912 0.86323 0.81913 0.77677 0.73612;%Density (kg/m^3)
        288.2 284.9  281.7  278.4  275.2  271.9   268.7   265.4   262.2   258.9   255.7  ;%Temperature (°K)
       101325 95460  89874  84555  79495  74682   70108   65764   61640   57728   54019  ;%Pressure (N/m^2)
        340.3 338.4  336.4  334.5  332.5  330.6   328.6   326.6   324.6   332.6   320.5 ];%Speed of sound (m/s)
        % Temperature
        hT(3,:) = hT(3,:) - 273.15; % °C
    else
    % U.S. Standard atmosphere in English units:
    hT = [0.0 1000.0 2000.0 3000.0 4000.0 5000.0 6000.0 7000.0 8000.0 9000.0 10000.0;%Altitude (ft)
        2.377 2.3081 2.2409 2.1751 2.1109 2.0481 1.9868 1.9268 1.8683 1.8111 1.75330;%Density *10^3
        518.7 515.10 511.50 508.00 504.40 500.80 497.30 493.70 490.10 486.60 483.000;%Temperature (°R)
       2116.0 2041.0 1963.0 1879.0 1828.0 1761.0 1696.0 1633.0 1572.0 1513.0 1456.00;%Pressure (psf=lb/ft^2)
       1116.4 1112.6 1108.7 1104.0 1101.0 1097.1 1093.2 1089.2 1085.3 1081.4 1077.4];%Speed of sound (ft/s)
        % Density
        hT(2,:) = hT(2,:)*10^(-3);      % slugs/ft^3
        hT(2,:) = hT(2,:)*515.379;      % kg/m^3
        % Temperature
        hT(3,:) = hT(3,:)*5/9 - 273.15; % °C
        % Pressure
        hT(4,:) = hT(4,:)*47.88;        % N/m^2
        % Speed of sound
        hT(5,:) = hT(5,:)*0.3048;       % m/s
        % airspeed
        V = V*0.51444;                  % m/s
    end
    
    % Polynomial curve fitting for Density
    pD = polyfit(hT(1,:),hT(2,:),3);
    Rho = @(h) pD(1)*h^3 + pD(2)*h^2 + pD(3)*h + pD(4);
    % Polynomial curve fitting for Temperature
    pT = polyfit(hT(1,:),hT(3,:),3);
    T = @(h) pT(1)*h^3 + pT(2)*h^2 + pT(3)*h + pT(4);
    % Polynomial curve fitting for Pressure
    pP = polyfit(hT(1,:),hT(4,:),3);
    P = @(h) pP(1)*h^3 + pP(2)*h^2 + pP(3)*h + pP(4);
    % Polynomial curve fitting for Speed of sound
    pA = polyfit(hT(1,:),hT(5,:),3);
    a = @(h) pA(1)*h^3 + pA(2)*h^2 + pA(3)*h + pA(4);
    
    % air density (kg/m^3)
    rho = Rho(h); flowproperties(1) = rho;
    % dynamic pressure N/m^2
    q = rho*(V^2)/2; flowproperties(2) = q;
    % Temperature (°C)
    flowproperties(3) = T(h);
    % Pressure (N/m^2)
    flowproperties(4) = P(h);
    % mach number
    M = V/a(h); flowproperties(5) = M;
    % Pressure ratio
    flowproperties(6) = P(h)/P(0);
end