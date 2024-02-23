%pks11 
%Trajectory Calculation

t0 = 1;
t1 = 2;
t2 = 2;
theta0 = 0.5;
theta1 = 0;
theta2 = 0;
thetadot0 = 0;
thetadot1 = 0;
thetadot2 = 0;

Traj_Matrix = [[1,t0,t0^2,t0^3];
                [0, 1,2*t0,3*t0^2];
                [1,t1,t1^2,t1^3];
                [0,1,2*t1, 3*t1^2]];

q =  (inv(Traj_Matrix))*[theta0;thetadot0;theta1;thetadot1];

a0 = q(1);
a1 = q(2);
a2 = q(3);
a3 = q(4);