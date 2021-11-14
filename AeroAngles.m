%% PartC Modified AeroAngles%%
function [V,a,b] = AeroAngles(u,v,w)
    V = sqrt(u.^2 + v.^2 + w.^2);
    a = atan2(w,u);
%     a = atan(w./abs(u));
    b = asin(v./V);
end