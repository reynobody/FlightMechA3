% AERO3560 Assignment 1
% Transformaton cosine matrices - Cz

function mat=Cz(psi)

psi = deg2rad(psi); % Converts the input from degrees to radians

mat = [cos(psi)  sin(psi) 0;          
       -sin(psi) cos(psi) 0;
        0        0        1 ];

end
