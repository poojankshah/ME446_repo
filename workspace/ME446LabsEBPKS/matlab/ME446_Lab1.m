%pks11 %elliana2
%ME446

%Finding Homogeneous transformation matrix syymobilically

clear all;


L1 = 0.254;
L2 = 0.254;
L3 = 0.254;

syms theta1 theta2 theta3;



h01 = [[cos(theta1) 0 -sin(theta1) 0];
    [sin(theta1) 0 cos(theta1) 0];
    [0 -1 0 0.254];
    [0 0 0 1]];

h12 = [[cos(theta2) -sin(theta2) 0 0.254*cos(theta2)];
    [sin(theta2) cos(theta2) 0 0.254*sin(theta2)];
    [0 0 1 0];
    [0 0 0 1]];

h23 = [[cos(theta3) -sin(theta3) 0 0.254*cos(theta3)];
    [sin(theta3) cos(theta3) 0 0.254*sin(theta3)];
    [0 0 1 0];
    [0 0 0 1]];

h03 = h01*h12*h23;

vpa(simplify(h03));

syms theta1m theta2m theta3m;
theta1 = theta1m;
theta2 = theta2m - sym(pi)/2;
theta3 = -1*theta2m + theta3m + sym(pi)/2;

h01_update =  [[cos(theta1) 0 -sin(theta1) 0];
    [sin(theta1) 0 cos(theta1) 0];
    [0 -1 0 0.254];
    [0 0 0 1]];

h12_update = [[cos(theta2) -sin(theta2) 0 0.254*cos(theta2)];
    [sin(theta2) cos(theta2) 0 0.254*sin(theta2)];
    [0 0 1 0];
    [0 0 0 1]];
h23_update = [[cos(theta3) -sin(theta3) 0 0.254*cos(theta3)];
    [sin(theta3) cos(theta3) 0 0.254*sin(theta3)];
    [0 0 1 0];
    [0 0 0 1]];

h03_update = h01_update*h12_update*h23_update;

h03_update = vpa(simplify(h03_update));

