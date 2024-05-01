%pks11
%Trying old lab
L1 = 0.254;
L2 = 0.254;
L3 = 0.254;

p1 = 0.03;
p2 = 0.0128;
p3 = 0.0076;
p4 = 0.0753;
p5 = 0.0298;

g = 9.81;

syms theta1 theta2 theta3;
syms theta1dot theta2dot theta3dot;
syms theta1dotdot theta2dotdot theta3dotdot;

thetadotdot = [theta2dotdot;theta3dotdot];
thetadot = [theta2dot;theta3dot];
theta = [theta2;theta3];

D = [p1,-p3*sin(theta3 - theta2);
    -p3*sin(theta3 - theta2), p2];

C = [0,-p3*cos(theta3-theta2)*theta3dot; p3*cos(theta3-theta2)*theta2dot, 0];

G = [-p4*g*sin(theta2);-p5*g*cos(theta3)];

tau = D*thetadotdot + C*thetadot + G;

tau = simplify(tau);
