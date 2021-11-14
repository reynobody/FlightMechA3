% AERO3560 Assignment 1
% Transformaton cosine matrices - Cx

function mat=Cx(phi) 

phi = deg2rad(phi); % Converts the input from degrees to radians

mat = [1 0          0;          
       0 cos(phi)   sin(phi);
       0 -sin(phi)  cos(phi)];

end