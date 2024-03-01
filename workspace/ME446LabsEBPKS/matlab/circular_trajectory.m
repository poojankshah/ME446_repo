%pks11
%circular trajectory
% L = length(0.35:0.001:0.45);
% X = eye(L,1);
% X0 = 0.45;
% Y = eye(L,1);
% for i = 1:L
% X(i) = X0 + i*0.001;
% Y(i) = abs(sqrt(-1*(X(i))^2 + 0.9*X(i) - 0.1925));
% 
% end


T =5;
dt = 0.001;
L_step = 5/0.001;
r = 0.1;
t = [0:dt:T];

X = eye(L_step,1);
Y = eye(L_step,1);

theta1_calc = eye(L_step,1);
theta2_calc = eye(L_step,1);
theta3_calc = eye(L_step,1);
X0 = 0.45;
Y0 = 0;
L = 0.254;
for i =1 :L_step
    X(i) = X0 + r*cos(2*pi*t(i)/T);
    Y(i) = Y0 + r*sin(2*pi*t(i)/T);


x_endeffector = X(i);
y_endeffector = Y(i);
z_endeffector = 0.15;

theta1_calc(i) = atan(y_endeffector/x_endeffector);
theta3_calc(i) = acos((power((L- z_endeffector),2) + power(x_endeffector,2) + power(y_endeffector,2) - 2*power(L,2))/(2*power(L,2)));
theta2_calc(i) = atan2((L-z_endeffector),sqrt((power(x_endeffector,2) + power(y_endeffector,2)))) - (theta3_calc(i)/2);
end