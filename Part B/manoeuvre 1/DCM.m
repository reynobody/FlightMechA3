% AERO8560 - Assignment 3

function mat = DCM(quaternions)
q0 = quaternions(1);
q1 = quaternions(2);
q2 = quaternions(3);
q3 = quaternions(4);

l1 = q0^2+q1^2-q2^2-q3^2;
l2 = 2*(q1*q2+q0*q3);
l3 = 2*(q1*q3-q0*q2);

m1 = 2*(q1*q2-q0*q3);
m2 = q0^2-q1^2+q2^2-q3^2;
m3 = 2*(q2*q3+q0*q1);

n1 = 2*(q0*q2+q1*q3);
n2 = 2*(q2*q3-q1*q0);
n3 = q0^2-q1^2-q2^2+q3^2;

mat = [l1,l2,l3;m1,m2,m3;n1,n2,n3];

end