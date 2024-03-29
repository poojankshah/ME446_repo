
syms JT_11 JT_12 JT_13 JT_21 JT_22 JT_23 JT_31 JT_32 JT_33;
syms Rwn_11 Rwn_12 Rwn_13 Rwn_21 Rwn_22 Rwn_23 Rwn_31 Rwn_32 Rwn_33;
syms Kpxn Kpyn Kpzn Kdxn Kdyn Kdzn;
syms xtask_d ytask_d ztask_d vxtask_d vytask_d vztask_d x_endeffector y_endeffector z_endeffector vx_endeffector vy_endeffector vz_endeffector;



JT = [JT_11 JT_12 JT_13;
    JT_21 JT_22 JT_23;
    JT_31 JT_32 JT_33];

Rwn = [Rwn_11 Rwn_12 Rwn_13;
    Rwn_21 Rwn_22 Rwn_23;
    Rwn_31 Rwn_32 Rwn_33];

tau = JT*Rwn*([Kpxn 0 0;0 Kpyn 0; 0 0 Kpzn]*(Rwn.')*[xtask_d-x_endeffector;ytask_d-y_endeffector;ztask_d-z_endeffector] + [Kdxn 0 0;0 Kdyn 0;0 0 Kdzn]*(Rwn.')*[vxtask_d-vx_endeffector;vytask_d-vy_endeffector;vztask_d-vz_endeffector]);

tau = simplify(tau);

