%pks11
%circular trajectory
%pks11 
% T: Time taken for circular trajectory to be finished
% dt : step time
% L : Total number of steps
% t : from time 0 to T with dt
T =5; 
dt = 0.001;
L_step = T/dt;

t = [dt:dt:T]';

X = eye(L_step,1);
Y = eye(L_step,1);

theta1_calc = eye(L_step,1);
theta2_calc = eye(L_step,1);
theta3_calc = eye(L_step,1);
%pks11
%circle trajectory : center : (X0, Y0)
%radius r : 0.1 m
% L : Link length

X0 = 0.3;
Y0 = 0.1;
Z0 = 0.35;
r = 0.1;
L = 0.254;
for i =1 :L_step
    %pks11 : 1 step for cicular trajectory
    % x = x0 + rcoswt
    % y = y0 + rsinwt
    X(i) = X0 + r*cos(2*pi*t(i)/T);
    Y(i) = Y0 + r*sin(2*pi*t(i)/T);


x_endeffector = X(i);
y_endeffector = Y(i);
z_endeffector = Z0;
%inverse kinematics for calculation of thetas
theta1_calc(i) = atan(y_endeffector/x_endeffector);
theta3_calc(i) = acos((power((L- z_endeffector),2) + power(x_endeffector,2) + power(y_endeffector,2) - 2*power(L,2))/(2*power(L,2)));
theta2_calc(i) = atan2((L-z_endeffector),sqrt((power(x_endeffector,2) + power(y_endeffector,2)))) - (theta3_calc(i)/2);
end