%pks11 
%Trajectory Calculation coefficients

%Requirements for cubic trajectory : pks11
%waypoints
t0 = 1;
t1 = 2;
t2 = 2;
theta0 = 0.5;
theta1 = 0;
theta2 = 0;
thetadot0 = 0;
thetadot1 = 0;
thetadot2 = 0;

%deriving trajecoty matrix : first row : theta1(t)
%second row : theta1dot(t)
%thirdrow : theta2(t)
%4th row : thetadot(t)
Traj_Matrix = [[1,t0,t0^2,t0^3];
                [0, 1,2*t0,3*t0^2];
                [1,t1,t1^2,t1^3];
                [0,1,2*t1, 3*t1^2]];

%find coefficient of cubic trajectory
q =  (inv(Traj_Matrix))*[theta0;thetadot0;theta1;thetadot1];

a0 = q(1);
a1 = q(2);
a2 = q(3);
a3 = q(4);