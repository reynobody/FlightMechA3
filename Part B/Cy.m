% AERO3560 Assignment 1
% Transformaton cosine matrices - Cy

function mat = Cy(theta)

theta = deg2rad(theta); % Converts the input from degrees to radians

mat = [cos(theta)   0  -sin(theta);          
       0            1   0;
       sin(theta)   0   cos(theta)];

end
