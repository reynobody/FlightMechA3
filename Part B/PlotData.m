% Function to convert current states and controls into presentable units
% Generate plots of all variables
% Input
% - [u v w] in m/s
% - [p q r] in rad/s
% - [q0 q1 q2 q3] in quaternions
% - [x y z] in m
% - [dt de da dr] in rad/s

function [] = PlotData(time, X_mat, control)

u = X_mat(1,:);
v = X_mat(2,:);
w = X_mat(3,:);
p = rad2deg(X_mat(4,:));
q = rad2deg(X_mat(5,:));
r = rad2deg(X_mat(6,:));
quaternions = [X_mat(7,:); X_mat(8,:); X_mat(9,:); X_mat(10,:)];
x = X_mat(11,:);
y = X_mat(12,:);
z = X_mat(13,:);

dt = rad2deg(control(1,:));
de = rad2deg(control(2,:));
da = rad2deg(control(3,:));
dr = rad2deg(control(4,:));

% Initialise figure number
fc = 0;

% Plot u, v, w
fc = fc + 1;
figure(fc)
plot(time,u,'r',time,v,'b',time,w,'m')
grid minor
legend('u','v','w')

% Plot body rates p,q,r 
fc = fc + 1;
figure(fc)
plot(time,p,'r',time,q,'b',time,r,'m')
grid minor
legend('Roll','Pitch','Yaw')

% Plot euler angles (convert quaternions to euler angles first)
for i = 1:length(quaternions(1,:))
    vec(3,i) = q2e(quaternions(:,i));
end
fc = fc + 1;
figure(fc)
plot(time,vec(1,:),'r',time,vec(2,:),'b',time,vec(3,:),'m')
grid minor
legend('phi','theta','psi')

% Plot x, y, z
fc = fc + 1;
figure(fc)
plot(time,x,'r',time,y,'b',time,z,'m')
grid minor
legend('x','y','z')

% Plot control angles
fc = fc + 1;
figure(fc)
plot(time,dt,time,de,time,da,time,dr)
grid minor
legend('Delta t','Delta e','Delta a','Delta r')

