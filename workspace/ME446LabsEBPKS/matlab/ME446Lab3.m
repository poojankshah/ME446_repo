%pks11 %elliana2
%ME446

%Finding Homogeneous transformation matrix syymobilically

syms theta1motor theta2motor theta3motor;
syms xd yd zd;
syms Kpx Kdx Kpy Kdy Kpz Kdz;
syms Vxd Vyd Vzd;
syms Vx Vy Vz;


cosq1 = cos(theta1motor);
sinq1 = sin(theta1motor);
cosq2 = cos(theta2motor);
sinq2 = sin(theta2motor);
cosq3 = cos(theta3motor);
sinq3 = sin(theta3motor);
JT_11 = -0.254*sinq1*(cosq3 + sinq2);
JT_12 = 0.254*cosq1*(cosq3 + sinq2);
JT_13 = 0;
JT_21 = 0.254*cosq1*(cosq2 - sinq3);
JT_22 = 0.254*sinq1*(cosq2 - sinq3);
JT_23 = -0.254*(cosq3 + sinq2);
JT_31 = -0.254*cosq1*sinq3;
JT_32 = -0.254*sinq1*sinq3;
JT_33 = -0.254*cosq3;

x = 0.254*cosq1*(cosq3 + sinq2);
y = 0.254*sinq1*(cosq3 + sinq2);
z = 0.254*(1+cosq2-sinq3);

JT = [JT_11 JT_12 JT_13;
    JT_21 JT_22 JT_23;
    JT_31 JT_32 JT_33];

Simple_force = JT*[Kpx*(xd - x)+ Kdx*(Vxd - Vx);
[Kpy*(yd - y)+ Kdy*(Vyd - Vy)];
[Kpz*(zd - z)+ Kdz*(Vzd - Vz)]];

