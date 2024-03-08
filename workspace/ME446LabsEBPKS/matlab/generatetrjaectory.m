%%pks11
%%trajectory generation

function [theta,thetadot,thetadotdot] = generatetrjaectory(time)
%2 different gains : from 0 to 1s cubic trakectory
a0_1 = 0;
a1_1 = 0;
a2_1 = 1.5;
a3_1 = -1;
%from 1 to 2s cubic trajectory
a0_2 = -2;
a1_2 = 6;
a2_2 = -4.5;
a3_2 = 1;



L = length(time);
%initialising theta matrix
theta = eye(L,1);
thetadot = eye(L,1);
thetadotdot = eye(L,1);
for i = 1:L
t = time(i);
if(t < 0)
%corner case
    theta(i) = a0_1;
    thetadot(i) = a1_1;
    thetadotdot(i) = 2*a2_1;


elseif ( (t <=1))
    %theta and thetadot for time varing from 0 to 1s
    theta(i) = a0_1 + a1_1*t + a2_1*(t^2) + a3_1*(t^3);
    thetadot(i) = a1_1 + 2*a2_1*t + 3*a3_1*(t^2);
    thetadotdot(i) = 2*a2_1 + 6*a3_1*t;

elseif((t <=2))
    %theta and thetadot for time varing from 1 to 2s
    theta(i) = a0_2 + a1_2*t + a2_2*(t^2) + a3_2*(t^3);
    thetadot(i) = a1_2 + 2*a2_2*t + 3*a3_2*(t^2);
    thetadotdot(i) = 2*a2_2 + 6*a3_2*t;
else
    %theta and thetadot after 2s
    theta(i) = 0;
    thetadot(i) = 0;
    thetadotdot(i) = 0;
end
end


