#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.40;//0; //-0.37; pks11
float offset_Enc3_rad = 0.228; //0.27; pks11


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array,".my_arrs") //pks11 - adding theta2
float theta2array[100];

float theta3 = 0;

long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//pks11 initialisation
float  x_endeffector = 0.15;
float  y_endeffector = 0;
float  z_endeffector = 0.43;

float theta1_calc = 0;
float theta2_calc = 0;
float theta3_calc = 0;

float theta1_motor_calc = 0;
float theta2_motor_calc = 0;
float theta3_motor_calc = 0;

//pks11 Implmenting the IIR filter
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

//pks11 // theta desired and thetadot desired
//theta2_desired and theta3_desired
float theta1_desired = 0;
float theta2_desired = 0;
float theta3_desired = 0;

float theta1dot_desired = 0;
float theta1dotdot_desired = 0;
float theta2dot_desired = 0;
float theta2dotdot_desired = 0;
float theta3dot_desired = 0;
float theta3dotdot_desired = 0;

//tracking error
float e_theta1 = 0;
float e_theta2 = 0;
float e_theta3 = 0;

//integral

float Ik_theta1_1 = 0;
float Ik_theta1 = 0;
float Ik_theta2_1 = 0;
float Ik_theta2 = 0;
float Ik_theta3_1 = 0;
float Ik_theta3 = 0;

//tracking_error one step back
float e_theta1_1 = 0;
float e_theta2_1 = 0;
float e_theta3_1 = 0;


float L = 0.254; //pks11 // Link parameter in meters
//Pd controller gains
float Kp1 = 50; // new value
float Kp2 = 50;//New value from 12.5 to 50
float Kp3 = 75;//new value from 12.5 to 75

float Kd1 = 2; //new value
float Kd2 = 2; // New value from 1 to 2
float Kd3 = 2; //New value from 1 to 2

//PID controller
float Kp1_PD = 50;
float Kp2_PD = 50;
float Kp3_PD = 75;

float Kd1_PD = 2;
float Kd2_PD = 2;
float Kd3_PD = 2;

float Kp1_PID = 100;
float Kp2_PID = 200; //old 40 - tuning process
float Kp3_PID = 150; // old 50 - tuning process

float Kd1_PID = 2;
float Kd2_PID = 7; // old 2 - tuning process
float Kd3_PID = 7;

float Ki1_PID = 10; // old 5 - tuning process
float Ki2_PID = 75; // old 5 - tuning process
float Ki3_PID = 75; // old 7.5 - tuning process

float a0_1 = 0;
float a1_1 = 0;
float a2_1 = 0;
float a3_1 = 0;

float a0_2 = 0;
float a1_2 = 0;
float a2_2 = 0;
float a3_2 = 0;

float dt = 0.001; // 1ms
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;


float T = 5;



//pks11 lab3 defining friction parameters
float u_fric_1 = 0;
float minimum_v_1 = 0.1;
float slope_bw_1 = 3.6;
float viscous_p_1 = 0.2513;
float viscous_n_1 = 0.2477;
float coulomb_p_1 = 0.3637;
float coulomb_n_1 = 0.2948;
float fric_coeff_1 = 0.9;
//joint2
float u_fric_2 = 0;
float minimum_v_2 = 0.05;
float slope_bw_2 = 3.6;
float viscous_p_2 = 0.25;
float viscous_n_2 = 0.287;
float coulomb_p_2 = 0.4759;
float coulomb_n_2 = 0.5031;
float fric_coeff_2 = 0.75;
//joint3
float u_fric_3 = 0;
float minimum_v_3 = 0.05;
float slope_bw_3 = 3.6;
float viscous_p_3 = 0.1922;
float viscous_n_3 = 0.2132;
float coulomb_p_3 = 0.3521;//pks11 takking mean of original and continuous (0.53 + 0.17)/2
float coulomb_n_3 = 0.3548;//pks11 takking mean of original and continuous (0.51 + 0.19)/2
float fric_coeff_3 = 1;

// Lab starter code
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;


//TASK SPACE CONTROL
float Kpx_task = 800; //100
float Kpy_task = 800; //200
float Kpz_task = 800; // 200

float Kdx_task = 40; // 4
float Kdy_task = 60; // 6
float Kdz_task = 60; //6


float xtask_d = 0.15;
float ytask_d = 0;
float ztask_d = 0.43;
float vxtask_d = 0;
float vytask_d = 0;
float vztask_d = 0;
float vx_endeffector = 0;
float vy_endeffector = 0;
float vz_endeffector = 0;

float x_endeffector_old = 0;
float vx_endeffector_old1 = 0;
float vx_endeffector_old2 = 0;

float y_endeffector_old = 0;
float vy_endeffector_old1 = 0;
float vy_endeffector_old2 = 0;

float z_endeffector_old = 0;
float vz_endeffector_old1 = 0;
float vz_endeffector_old2 = 0;

float Fzcmd = -1; //giving force
float Kt = 6;

//Part 4 : Impedence Control
float Rwn_11  = 0;
float Rwn_12 = 0;
float Rwn_13 = 0;
float Rwn_21 = 0;
float Rwn_22 = 0;
float Rwn_23 = 0;
float Rwn_31 = 0;
float Rwn_32 = 0;
float Rwn_33 = 0;

float thetaz_r = 0; //for xN alligned with line
float thetax_r = 0;
float thetay_r = 0;

//float Kpxn = 200; //Kpx_task (nEED TO RETUNE) As it was not compliant
float Kpxn = 800; //Kpxn need to bbe retuned (earlier 200)
float Kpyn = 200; // Kpy_task
float Kpzn = 200; //Kpz_task

//float Kdxn = 4; // Kdx_task new
float Kdxn = 100; // maybe reduced a bit; earlier 4
float Kdyn = 6; // Kdy_task
//float Kdzn = 6; //Kdz_task
float Kdzn = 50; // maybe reduced a bit; earlier 6

//straight line motion
float t_total = 4;
float xa_taskd = 0.25;
float ya_taskd = 0;
float za_taskd = 0.35;

float xb_taskd = 0.35;
float yb_taskd = 0.1;
float zb_taskd = 0.35;
float t = 0;
int t_int = 0;

float t_try1 = 0;
int TIME = 2;

//pks11//
//writing or specifing structure of waypoints;

//waypoint description
//xdes,ydes,zdes : waypoint description
// tdes : global time to reach the waypoint

typedef struct{
    float xdes;
    float ydes;
    float zdes;
    float tdes;
    int controllerdesired;
} waypoint;


//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,2,1},{0.03254,0.34873,0.25254,3,1},{0.03013,0.35212,0.12072,4,2},{0.03013,0.35212,0.2455,5,2},{0.21149,0.1195,0.338,6.5,1},{0.38189,0.13049,0.28786,7.5,1},{0.39062,0.1228,0.209,9,1},{0.40735,0.10127,0.209,10.5,3},{0.41368,0.08619,0.209,13,3},{0.41451,0.07224,0.209,14,1},{0.40125,0.06105,0.209,15,1},{0.38721,0.06634,0.209,16,4},{0.36081,0.0722,0.209,17,4},{0.341,0.0749,0.209,18.5,4},{0.32409,0.0656,0.209,20,1},{0.3287,0.05087,0.209,21,1},{0.34217,0.03298,0.209,22,3},{0.3778,-0.01006,0.209,23,3},{0.3778,-0.01006,0.319,24,1},{0.24243,0.19306,0.319,25,1},{0.24243,0.19306,0.29423,26.5,1},{0.24243,0.19306,0.28223,27.5,5},{0.24243,0.19306,0.28223,29.5,5},{0.254,0,0.508,30.5,1}};
//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,1.5,1},{0.03254,0.34873,0.25254,2,1},{0.03013,0.35212,0.13072,2.2,2},{0.03013,0.35212,0.13072,2.7,2},{0.03013,0.35212,0.2455,3,2},{0.03013,0.35212,0.36,3.2,1},{0.22104,0.17271,0.3654,3.5,1},{0.39038,0.12639,0.211,3.8,1},{0.41004,0.089,0.211,4.3,3},{0.41226,0.07374,0.211,4.5,1},{0.41429,0.07739,0.211,4.6,1},{0.40582,0.06503,0.211,4.7,1},{0.3423,0.07322,0.211,4.9,4},{0.32454,0.0656,0.211,5.2,1},{0.3289,0.04675,0.211,5.3,1},{0.3778,-0.01006,0.211,5.6,3},{0.3778,-0.01006,0.319,5.8,1},{0.24243,0.19306,0.319,6.2,1},{0.24243,0.19306,0.33423,6.3,1},{0.24243,0.19306,0.28223,6.5,5},{0.24243,0.19306,0.28223,8.5,5},{0.24243,0.19306,0.39423,8.8,1},{0.148,0,0.43,9.2,1}};
//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,1.5,1},{0.03254,0.34873,0.25254,2,1},{0.03013,0.35212,0.13072,2.2,2},{0.03013,0.35212,0.13072,2.7,2},{0.03013,0.35212,0.2455,3,2},{0.03013,0.35212,0.36,3.2,1},{0.22104,0.17271,0.3654,3.5,1},{0.39038,0.12639,0.211,3.8,1},{0.41368,0.08619,0.211,4.3,3},{0.41226,0.07374,0.211,4.5,1}};
//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,1.5,1},{0.03254,0.34873,0.25254,2,1},{0.03013,0.35212,0.13072,2.2,2},{0.03013,0.35212,0.13072,2.7,2},{0.03013,0.35212,0.2455,3,2},{0.03013,0.35212,0.36,3.2,1},{0.22104,0.17271,0.3654,3.5,1},{0.39038,0.12639,0.211,3.8,1},{0.41349,0.09354,0.211,4.3,3},{0.41226,0.07374,0.211,4.5,7},{0.41353,0.07729,0.211,4.6,7},{0.40582,0.06503,0.211,4.7,7},{0.3423,0.07322,0.211,4.9,4},{0.32454,0.0656,0.211,5.2,1},{0.3289,0.04675,0.211,5.3,1},{0.3778,-0.01006,0.211,5.6,3},{0.3778,-0.01006,0.319,5.8,1},{0.24243,0.19306,0.319,6.2,1},{0.24243,0.19306,0.33423,6.3,1},{0.24243,0.19306,0.28223,6.5,5},{0.24243,0.19306,0.28223,8.5,5},{0.24243,0.19306,0.39423,8.8,1},{0.148,0,0.43,9.2,1}};
//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,1.5,1},{0.03254,0.34873,0.25254,2,1},{0.03013,0.35212,0.13072,2.2,2},{0.03013,0.35212,0.13072,2.7,2},{0.03013,0.35212,0.2455,3,2},{0.03013,0.35212,0.36,3.2,1},{0.22104,0.17271,0.3654,3.5,1},{0.39038,0.12639,0.311,3.8,1},{0.41004,0.0892,0.311,4.3,3}};


// slow trajectory
waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,2,1},{0.03254,0.34873,0.25254,3,1},{0.03708,0.35456,0.12072,4,2},{0.03708,0.35456,0.12072,4.65,2},{0.03708,0.35456,0.2055,5.15,2},{0.19532,0.16614,0.3633,6.5,1},{0.39038,0.12639,0.211,9,1},{0.40735,0.10127,0.211,10.5,3},{0.41368,0.08619,0.211,13,3},{0.41451,0.07224,0.211,14,1},{0.40125,0.06105,0.211,15,1},{0.38721,0.06634,0.211,16,4},{0.36081,0.0722,0.211,17,4},{0.341,0.0749,0.211,18.5,4},{0.32409,0.0656,0.211,20,1},{0.3287,0.05087,0.211,21,1},{0.34217,0.03298,0.211,21.75,3},{0.3778,-0.01006,0.211,22.5,3},{0.392,-0.01655,0.211,23,1},{0.3778,-0.01006,0.319,24,1},{0.24243,0.19306,0.319,25,1},{0.24243,0.19306,0.29423,26.5,1},{0.24243,0.19306,0.28223,27.5,5},{0.24243,0.19306,0.28223,29.5,5},{0.254,0,0.508,30.5,1}};
//waypoint point[] = {{0.148,0,0.43,1,1},{0,0.34,0.35,2,1},{0.03254,0.34873,0.25254,3,1},{0.03708,0.35456,0.12072,4,2},{0.03708,0.35456,0.12072,4.5,2},{0.03708,0.35456,0.2255,5,2}};


//pks11 waypoint specification
int num_waypoint = 0; // total number of waypoint
int waypoint_index = 0; // existing waypoint index
int controllerdesired = 1;
//pks11
float time_global = 0; // global time clock
float time_traj = 0; // time taken to reach i-1 waypoint to ith waypoint
float time_local = 0; // local time for travelling from i-1 waypoint to i waypoint
float temp = 0;
float lastpoint_flag = 0;
float vx_taskd_k_1 = 0;
float vy_taskd_k_1 = 0;
float vz_taskd_k_1 = 0;

float vx_taskd_k_2 = 0;
float vy_taskd_k_2 = 0;
float vz_taskd_k_2 = 0;

void mains_code(void);

//
// Main
//
void main(void)
{

    mains_code();
}



// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    //*tau1 = 0; //pks11 // torques providing to 0
    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //float Kpxn = 200; //Kpx_task (nEED TO RETUNE) As it was not compliant
    float Kpxn = 800; //Kpxn need to bbe retuned (earlier 200)
    float Kpyn = 200; // Kpy_task
    float Kpzn = 200; //Kpz_task

    //float Kdxn = 4; // Kdx_task new
    float Kdxn = 100; // maybe reduced a bit; earlier 4
    float Kdyn = 6; // Kdy_task
    //float Kdzn = 6; //Kdz_task
    float Kdzn = 50; // maybe reduced a bit; earlier 6

    //TASK SPACE CONTROL
    float Kpx_task = 800; //100
    float Kpy_task = 800; //200
    float Kpz_task = 800; // 200

    float Kdx_task = 40; // 4
    float Kdy_task = 60; // 6
    float Kdz_task = 60; //6
    thetaz_r = 0;



    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    //pks11
    //global time
    time_global = mycount/1000.0;

    // number of waypoints pks11//
    num_waypoint = sizeof(point)/sizeof(point[0]);
    temp = sizeof(point);

    //starting to first waypoint
    if(waypoint_index == 0)
    {
        time_local = time_global;
        time_traj = point[waypoint_index].tdes;
        xtask_d = x_endeffector + time_local*(point[waypoint_index].xdes - x_endeffector)/time_traj;
        ytask_d = y_endeffector + time_local*(point[waypoint_index].ydes - y_endeffector)/time_traj;
        ztask_d = z_endeffector + time_local*(point[waypoint_index].zdes - z_endeffector)/time_traj;

        vxtask_d = (point[waypoint_index].xdes - x_endeffector)/time_traj;

        vytask_d = (point[waypoint_index].ydes - y_endeffector)/time_traj;
        vztask_d = (point[waypoint_index].zdes - z_endeffector)/time_traj;
        controllerdesired = 1;
    }


    if(time_global > point[waypoint_index].tdes && waypoint_index < num_waypoint-1)
    {
        time_traj = point[waypoint_index+1].tdes - point[waypoint_index].tdes;
        time_local = (time_global - point[waypoint_index].tdes);

        xtask_d = point[waypoint_index].xdes + time_local*(point[waypoint_index+1].xdes - point[waypoint_index].xdes)/time_traj;
        ytask_d = point[waypoint_index].ydes + time_local*(point[waypoint_index+1].ydes - point[waypoint_index].ydes)/time_traj;
        ztask_d = point[waypoint_index].zdes + time_local*(point[waypoint_index+1].zdes - point[waypoint_index].zdes)/time_traj;

        vxtask_d = (point[waypoint_index+1].xdes - point[waypoint_index].xdes)/time_traj;
        vytask_d = (point[waypoint_index+1].ydes - point[waypoint_index].ydes)/time_traj;
        vztask_d = (point[waypoint_index+1].zdes - point[waypoint_index].zdes)/time_traj;

        controllerdesired = point[waypoint_index].controllerdesired;


        if(fabs(time_global - point[waypoint_index+1].tdes) <= 0.0005)
        {
            waypoint_index  = waypoint_index + 1;
            lastpoint_flag = 1;

        }
    }

    if(waypoint_index == num_waypoint-1)
    {

        xtask_d = point[waypoint_index].xdes;
        ytask_d = point[waypoint_index].ydes;
        ztask_d = point[waypoint_index].zdes;
        vxtask_d = 0;
        vytask_d = 0;
        vztask_d = 0;
        lastpoint_flag = 2;
        controllerdesired = point[waypoint_index].controllerdesired;


    }

    //filtering the velocity //pks11
    //vxtask_d = (vxtask_d + vx_taskd_k_1 + vx_taskd_k_2)/3.0;
    //vytask_d = (vytask_d + vy_taskd_k_1 + vy_taskd_k_2)/3.0;
    //vztask_d = (vztask_d + vz_taskd_k_1 + vz_taskd_k_2)/3.0;





    //pks11
    //Trajectory Generation Coefficient
    //Rising theta = 0 to theta = 0.5 in 0-1 second
    //    a0_1 = 0;
    //    a1_1 = 0;
    //    a2_1 = 1.5;
    //    a3_1 = -1;
    //
    //    //declining, theta - 0.5 to theta = 0 in 1-2 second
    //    a0_2 = -2;
    //    a1_2 = 6;
    //    a2_2 = -4.5;
    //    a3_2 = 1;
    // pks11 : writing theta, thetadot, thetadotdot
    //    if (mycount <=1000){
    //        theta1_desired = a0_1 + a1_1*mycount*dt + a2_1*(pow(mycount*dt,2)) + a3_1*(pow(mycount*dt,3));
    //        theta1dot_desired =   a1_1 + 2*a2_1*(mycount*dt) + 3*a3_1*(pow(mycount*dt,2));
    //        theta1dotdot_desired =  2*a2_1 + 6*a3_1*mycount*dt;
    //    }
    //
    //    if((mycount>1000) && (mycount <=2000)){
    //        theta1_desired = a0_2 + a1_2*mycount*dt + a2_2*(pow(mycount*dt,2)) + a3_2*(pow(mycount*dt,3));
    //        theta1dot_desired =  a1_2 + 2*a2_2*(mycount*dt) + 3*a3_2*(pow(mycount*dt,2));
    //        theta1dotdot_desired =  2*a2_2 + 6*a3_2*mycount*dt;
    //    }
    //
    //    if(mycount > 2000){
    //        theta1_desired = 0;
    //        theta1dot_desired = 0;
    //        theta1dotdot_desired = 0;
    //    }



    //Since we are using same for theta2 theta3
    //    theta2_desired = theta1_desired;
    //    theta2dot_desired = theta1dot_desired;
    //    theta2dotdot_desired = theta1dotdot_desired;
    //
    //    theta3_desired = theta1_desired;
    //    theta3dot_desired = theta1dot_desired;
    //    theta3dotdot_desired = theta1dotdot_desired;


    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    //pks11 adding a variable
    theta3 = theta3motor;
    ////
    //    //pks11 Implmenting the theta dot filters
    //    // filter type : IIR
    //    //theta1dot :: Omega1
    //    Omega1 = (theta1motor - Theta1_old)/0.001;
    //    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    //    Theta1_old = theta1motor;
    //    //order matters here. If you don't update in correct order
    //    // Omega1_Old2 will get wrong value instead of getting Omega1_Old1
    //    //State update for filters
    //    Omega1_old2 = Omega1_old1;
    //    Omega1_old1 = Omega1;
    //
    //    //theta2dot : Omega2
    //    Omega2 = (theta2motor - Theta2_old)/0.001;
    //    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    //    Theta2_old = theta2motor;
    //    //order matters here. If you don't update in correct order
    //    // Omega2_Old2 will get wrong value instead of getting Omega2_Old1
    //    //State update for filters
    //    Omega2_old2 = Omega2_old1;
    //    Omega2_old1 = Omega2;
    //
    //    //theta3dot : Omega3
    //    Omega3 = (theta3motor - Theta3_old)/0.001;
    //    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    //    Theta3_old = theta3motor;
    //    //order matters here. If you don't update in correct order
    //    // Omega3_Old2 will get wrong value instead of getting Omega3_Old1
    //    //State update for filters
    //    Omega3_old2 = Omega3_old1;
    //    Omega3_old1 = Omega3;
    //
    //    /// pks11 Tracking errors calculation
    //    //trackingerror_theta1
    //
    //    e_theta1 = theta1_desired - theta1motor;
    //
    //    //trackingerror_theta2
    //    e_theta2 = theta2_desired - theta2motor;
    //
    //    //trackingerror_theta3
    //    e_theta3 = theta3_desired - theta3motor;
    //
    //    //pid controller Adaptive gains!
    //    if(fabs(e_theta1) < 0.08)
    //    {
    //        //pks11
    //        //*tau1 = Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 - Kd1_PID*(Omega1);  //pks11 : Older PID
    //        //*tau1 = J1*theta1dotdot_desired + Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 + Kd1_PID*(theta1dot_desired - Omega1); // pks11 : Feedforward control (PID)
    //        //Feedforwad Control : we are trying to provide the information of the future : It can be done from providing some information about the acceleration
    //        *tau1 = Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 - Kd1_PID*(Omega1);
    //        Ik_theta1 = Ik_theta1_1 + (e_theta1 + e_theta1_1)*0.001;
    //    }
    //    else
    //    {
    //        Ik_theta1 = 0;
    //        //*tau1 =J1*theta1dotdot_desired + Kp1_PD*(theta1_desired - theta1motor) + Kd1_PD*(theta1dot_desired - Omega1);// pks11 : Feedforward control (PD)
    //        *tau1 = Kp1_PID*(theta1_desired - theta1motor) - Kd1_PID*(Omega1);
    //    }
    //
    //    //theta2 control
    //    if(fabs(e_theta2) < 0.08)
    //    {
    //
    //        //*tau2 = J2*theta2dotdot_desired + Kp2_PID*(theta2_desired - theta2motor) + Ki2_PID*Ik_theta2 + Kd2_PID*(theta2dot_desired - Omega2); // pks11 : Feedforward control (PID)
    //        *tau2 = Kp2_PID*(theta2_desired - theta2motor) + Ki2_PID*Ik_theta2 - Kd2_PID*(Omega2);
    //        Ik_theta2 = Ik_theta2_1 + (e_theta2 + e_theta2_1)*0.001;
    //    }
    //    else
    //    {
    //        Ik_theta2 = 0;
    //        //*tau2 = J2*theta2dotdot_desired + Kp2_PD*(theta2_desired - theta2motor) + Kd2_PD*(theta2dot_desired - Omega2); // pks11 : Feedforward control (PD)
    //        *tau2 = Kp2_PID*(theta2_desired - theta2motor) - Kd2_PID*(Omega2);
    //    }
    //
    //    //theta3 control
    //    if(fabs(e_theta3) < 0.08)
    //    {
    //        //*tau3 = J3*theta3dotdot_desired + Kp3_PID*(theta3_desired - theta3motor) + Ki3_PID*Ik_theta3 + Kd3_PID*(theta3dot_desired - Omega3); // pks11 : Feedforward control (PID)
    //        *tau3 = Kp3_PID*(theta3_desired - theta3motor) + Ki3_PID*Ik_theta3 - Kd3_PID*(Omega3);
    //        Ik_theta3 = Ik_theta3_1 + (e_theta3 + e_theta3_1)*0.001;
    //    }
    //
    //    else
    //    {
    //        Ik_theta3 = 0;
    //        //*tau3 = J3*theta3dotdot_desired + Kp3_PD*(theta3_desired - theta3motor) + Kd3_PD*(theta3dot_desired - Omega3); // pks11 : Feedforward control (PD)
    //        *tau3 = Kp3_PID*(theta3_desired - theta3motor) - Kd3_PID*(Omega3);
    //    }


    //// PD control
    //pks11 calculating the taus // pd controller
    //    *tau1 = Kp1*(theta1_desired - theta1motor) - Kd1*(Omega1);
    //    *tau2 = Kp2*(theta2_desired - theta2motor) - Kd2*(Omega2);
    //    *tau3 = Kp3*(theta3_desired - theta3motor) - Kd3*(Omega3);

    //pks11
    //Saturation block
    // this saturation block is designed such that it will make sure to stop integrating when torque saturation limit is reached
    // NOTE : we are not zeroing the integration, however, we are stopping to add more values
    // Antiwindup
    if(*tau1 < -5)
    {
        *tau1 = -5;
        Ik_theta1 = Ik_theta1_1;
    }

    if(*tau1 > 5)
    {
        *tau1 = 5;
        Ik_theta1 = Ik_theta1_1;
    }

    if(*tau2 < -5)
    {
        *tau2 = -5;
        Ik_theta2 = Ik_theta2_1;

    }

    if(*tau2 > 5)
    {
        *tau2 = 5;
        Ik_theta2 = Ik_theta2_1;
    }

    if(*tau3 < -5)
    {
        *tau3 = -5;
        Ik_theta3 = Ik_theta3_1;
    }

    if(*tau3 > 5)
    {
        *tau3 = 5;
        Ik_theta3 = Ik_theta3_1;
    }


    //pks11 lab3 new code
    //part 1
    //adding friction coefficient to joints to reduce stiffness
    //joint 1 : *tau1, Omega1
    //pks11 lab3 defining friction parameters
    /*    float minimum_v_1 = 0.1;
    float slope_bw_1 = 3.6;
    float viscous_p_1 = 0.2513;
    float viscous_n_1 = 0.2477;
    float coulomb_p_1 = 0.3637;
    float coulomb_n_1 = 0.2948;*/
    if(Omega1 > minimum_v_1)
    {
        u_fric_1 = viscous_p_1*Omega1 + coulomb_p_1;
    }
    else if(Omega1 < -1*minimum_v_1)
    {
        u_fric_1 = viscous_n_1*Omega1 - coulomb_n_1;
    }
    else
    {
        u_fric_1 = slope_bw_1*Omega1;
    }

    //joint2
    if(Omega2 > minimum_v_2)
    {
        u_fric_2 = viscous_p_2*Omega2 + coulomb_p_2;
    }
    else if(Omega2 < -1*minimum_v_2)
    {
        u_fric_2 = viscous_n_2*Omega2 - coulomb_n_2;
    }
    else
    {
        u_fric_2 = slope_bw_2*Omega2;
    }



    //joint3
    if(Omega3 > minimum_v_3)
    {
        u_fric_3 = viscous_p_3*Omega3 + coulomb_p_3;
    }
    else if(Omega3 < -1*minimum_v_3)
    {
        u_fric_3 = viscous_n_3*Omega3 - coulomb_n_3;
    }
    else
    {
        u_fric_3 = slope_bw_3*Omega3;
    }

    //pks11 lab3 // updating torques
    *tau1 = fric_coeff_1*u_fric_1;
    *tau2 = fric_coeff_2*u_fric_2;
    *tau3 = fric_coeff_3*u_fric_3;

    //PD control for TASK SPACE CALCULATION
    //pks11 adding a calculation of h03 with motor angles
    //Steps Folllowed : Calculated forward kinemtics from the DH table and provide the coordinates of endeffector which is [0;0;0] in frame 3
    // Measured the relationship between theta angles and joint motors angles.
    //Relationship : theta2DH = theta2m - pi/2;
    // theta3DH = -theta2m + theta3m + pi/2;
    x_endeffector = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y_endeffector = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z_endeffector = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

    //pks11 - lab3 calculation of xdot, ydot zdot by applying IIR filter
    vx_endeffector = (x_endeffector - x_endeffector_old)/0.001;
    vx_endeffector = (vx_endeffector + vx_endeffector_old1 + vx_endeffector_old2)/3.0;
    x_endeffector_old = x_endeffector;

    //State update for filters
    vx_endeffector_old2 = vx_endeffector_old1;
    vx_endeffector_old1 = vx_endeffector;

    vy_endeffector = (y_endeffector - y_endeffector_old)/0.001;
    vy_endeffector = (vy_endeffector + vy_endeffector_old1 + vy_endeffector_old2)/3.0;
    y_endeffector_old = y_endeffector;

    //State update for filters
    vy_endeffector_old2 = vy_endeffector_old1;
    vy_endeffector_old1 = vy_endeffector;

    vz_endeffector = (z_endeffector - z_endeffector_old)/0.001;
    vz_endeffector = (vz_endeffector + vz_endeffector_old1 + vz_endeffector_old2)/3.0;
    z_endeffector_old = z_endeffector;

    //State update for filters
    vz_endeffector_old2 = vz_endeffector_old1;
    vz_endeffector_old1 = vz_endeffector;


    // Jacobian Transpose calculation
    // We have used Jacobian for CRS robot
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

    //    float Kpx_task = 0;
    //    float Kpy_task = 0;
    //    float Kpz_task = 0;
    //
    //    float Kdx_task = 0;
    //    float Kdy_task = 0;
    //    float Kdz_task = 0;
    //
    //
    //    float xtask_d = 0;
    //    float ytask_d = 0;
    //    float ztask_d = 0;
    //    float vxtask_d = 0;
    //    float vytask_d = 0;
    //    float vztask_d = 0;


    // Lab 3 part 2,3
    // From part 2, we get approximately friction values and friction compensation;
    // Thus, PD control for task space control  is implemented by adding this friction coefficient to older task space control
    // task space control : torque = [Jtranspose*Force]
    //FOrce = Kp*(position error) + Kd*(Velocity Error) // tuning for position and velocity
    //friction coeff factor calculation : (FOR AVOIDING BOUNCING OF ROBOT)
    // Tried to Kp Kd of Z 0 and see the bouncing robot;  thus from that evalaute the friction factor (verify) : So that bouncing is not visible

    if(controllerdesired == 1)
    {

        *tau1 = fric_coeff_1*u_fric_1 +  JT_11*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_12*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_13*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector));
        *tau2 = fric_coeff_2*u_fric_2 +  JT_21*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_22*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_23*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector));
        *tau3 = fric_coeff_3*u_fric_3 +  JT_31*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_32*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_33*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector));
    }
    //pks11
    // Adding feedforward element :
    // Kt is assumed to be 6
    // Giving the negative force compensation varies from 0 to -20
    // By implementing this as feedforward component  (Jt*[F/kt]); we can feel the force by holding the robot at particular Z direction AND changing this Ft
    //    *tau1 = fric_coeff_1*u_fric_1 +  JT_11*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_12*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_13*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_13*Fzcmd/Kt;
    //    *tau2 = fric_coeff_2*u_fric_2 +  JT_21*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_22*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_23*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_23*Fzcmd/Kt;
    //    *tau3 = fric_coeff_3*u_fric_3 +  JT_31*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_32*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_33*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_33*Fzcmd/Kt;

    //Defining the rotation matrix
    //Lab 3 part 4
    //float thetaz_r = pi/4;
    //float thetax_r = 0;
    //float thetay_r = 0;
    // Rotation Matrix :
    //frame N, and the world coordinate frame of the robot frame W. Frame N is found by rotating (thetaZ) about the z axis and then rotating thetax about the x axis and then thetay
    // about the y axis. Rwn_zxy :
    Rwn_11 = cos(thetaz_r)*cos(thetay_r) - sin(thetaz_r)*sin(thetax_r)*sin(thetay_r);
    Rwn_12 = -1*sin(thetaz_r)*cos(thetax_r);
    Rwn_13 = cos(thetaz_r)*sin(thetay_r) + sin(thetaz_r)*sin(thetax_r)*cos(thetay_r);
    Rwn_21 = sin(thetaz_r)*cos(thetay_r) + cos(thetaz_r)*sin(thetax_r)*sin(thetay_r);
    Rwn_22 = cos(thetaz_r)*cos(thetax_r);
    Rwn_23 = sin(thetaz_r)*sin(thetay_r) - cos(thetaz_r)*sin(thetax_r)*cos(thetay_r);
    Rwn_31 = -1*cos(thetax_r)*sin(thetay_r);
    Rwn_32 = sin(thetax_r);
    Rwn_33 = cos(thetax_r)*cos(thetay_r);

    //pks11
    //impedence control
    //Lab 3 Ex 4
    //Impedence control consists of : Friction Compensation; Force control in Xn,Yn,Zn (This is calculated with the help of Position error and velocity error)
    // Now, important thing to note is; Rotational frame : For example, Rwn_ij is i th row and jth column of Rotation matrix of N frame with World frame as reference.
    // We want to have impedence control in the Nth frame and hence we are utilising rotational matrix to evalaute impedence control in XN, YN,ZN
    // NOTE : position error and velocity errors are defined in the world frame; that means it's going to follow trajectory based on world frame coordinates
    // Now, if you weaken your axis in any direction, it will be with respect to N frame not world frame.
    // We take help of Rotation matrix to convert from world coordinate frame to normal coordinate frame


    //    *tau1 = fric_coeff_1*u_fric_1 - (JT_11*Rwn_11 + JT_12*Rwn_21 + JT_13*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_11*Rwn_12 + JT_12*Rwn_22 + JT_13*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_11*Rwn_13 + JT_12*Rwn_23 + JT_13*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
    //    *tau2 = fric_coeff_2*u_fric_2 - (JT_21*Rwn_11 + JT_22*Rwn_21 + JT_23*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_21*Rwn_12 + JT_22*Rwn_22 + JT_23*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_21*Rwn_13 + JT_22*Rwn_23 + JT_23*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
    //    *tau3 = fric_coeff_3*u_fric_3 - (JT_31*Rwn_11 + JT_32*Rwn_21 + JT_33*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_31*Rwn_12 + JT_32*Rwn_22 + JT_33*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_31*Rwn_13 + JT_32*Rwn_23 + JT_33*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));


    //pks11
    //lab3 exercise : 5
    //writing the time function for the staright line trajectory for going to-fro;
    // Time taken for straightline : 2s // FIXING THESE
    // total time for to-fro : 2+2 = 4s (freqeuncy of wave : 1/4 = 0.25Hz)


    //    t_int = mycount/(TIME*1000.0); //returning the integer value (2500/2000 = 1)
    //    t_try1 = fmod((mycount/1000.0) , TIME); // this is be our time variable varing from 0 to 2;
    //
    //    if(t_int % 2 == 0) //(going from point A to point B in TIME T) // (0 to 1; 2 to 3; ....)
    //
    //    {
    //        xtask_d = (xb_taskd-xa_taskd)*(t_try1)/TIME + xa_taskd; // (xb-xa)*(time/TOTAL TIME); same for y and z
    //        ytask_d = (yb_taskd-ya_taskd)*(t_try1)/TIME + ya_taskd;
    //        ztask_d = (zb_taskd-za_taskd)*(t_try1)/TIME + za_taskd;
    //        vxtask_d = (xb_taskd - xa_taskd)/TIME; // Defining the velocity for the better control; te trajectory is straight line thus; velocity will be slope of that line
    //        vytask_d = (yb_taskd - ya_taskd)/TIME;
    //        vztask_d = (zb_taskd - za_taskd)/TIME;
    //    }
    //
    //    if(t_int % 2 !=0) //(going from Point B to point A in TIME T) // (1 to 2; 3 to 4;....)
    //    {
    //        xtask_d = (xa_taskd-xb_taskd)*(t_try1)/TIME + xb_taskd;
    //        ytask_d = (ya_taskd-yb_taskd)*(t_try1)/TIME + yb_taskd;
    //        ztask_d = (za_taskd-zb_taskd)*(t_try1)/TIME + zb_taskd;
    //        vxtask_d = (xa_taskd - xb_taskd)/TIME;
    //        vytask_d = (ya_taskd - yb_taskd)/TIME;
    //        vztask_d = (za_taskd - zb_taskd)/TIME;
    //    }



    //pks11
    //Impedence Control Implementation :
    //Lab 3 Ex 5
    //Impedence control consists of : Friction Compensation; Force control in Xn,Yn,Zn (This is calculated with the help of Position error and velocity error)
    // Now, important thing to note is; Rotational frame : For example, Rwn_ij is i th row and jth column of Rotation matrix of N frame with World frame as reference.
    // We want to have impedence control in the Nth frame and hence we are utilising rotational matrix to evalaute impedence control in XN, YN,ZN
    // NOTE : position error and velocity errors are defined in the world frame; that means it's going to follow trajectory based on world frame coordinates
    // Now, if you weaken your axis in any direction, it will be with respect to N frame not world frame.
    // We take help of Rotation matrix to convert from world coordinate frame to normal coordinate frame


    if(controllerdesired == 2)
    {
        Kpxn = 0;
        Kpyn = 0;
        Kpzn = 800; //Lazy trajctory
        //Kpzn = 200; //Fast trajectory

        Kdxn = 0;
        Kdyn = 0;
        Kdzn = 100; //Lazy trajectory
        //Kdzn = 50; //fast trajectory
        *tau1 = fric_coeff_1*u_fric_1 - (JT_11*Rwn_11 + JT_12*Rwn_21 + JT_13*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_11*Rwn_12 + JT_12*Rwn_22 + JT_13*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_11*Rwn_13 + JT_12*Rwn_23 + JT_13*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau2 = fric_coeff_2*u_fric_2 - (JT_21*Rwn_11 + JT_22*Rwn_21 + JT_23*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_21*Rwn_12 + JT_22*Rwn_22 + JT_23*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_21*Rwn_13 + JT_22*Rwn_23 + JT_23*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau3 = fric_coeff_3*u_fric_3 - (JT_31*Rwn_11 + JT_32*Rwn_21 + JT_33*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_31*Rwn_12 + JT_32*Rwn_22 + JT_33*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_31*Rwn_13 + JT_32*Rwn_23 + JT_33*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
    }


    if(controllerdesired == 3)
    {
        Kpxn = 600;
        Kpyn = 0;
        Kpzn = 600;

        Kdxn = 100;
        Kdyn = 0;
        Kdzn = 100;

        thetaz_r = -0.927;

        Rwn_11 = cos(thetaz_r)*cos(thetay_r) - sin(thetaz_r)*sin(thetax_r)*sin(thetay_r);
        Rwn_12 = -1*sin(thetaz_r)*cos(thetax_r);
        Rwn_13 = cos(thetaz_r)*sin(thetay_r) + sin(thetaz_r)*sin(thetax_r)*cos(thetay_r);
        Rwn_21 = sin(thetaz_r)*cos(thetay_r) + cos(thetaz_r)*sin(thetax_r)*sin(thetay_r);
        Rwn_22 = cos(thetaz_r)*cos(thetax_r);
        Rwn_23 = sin(thetaz_r)*sin(thetay_r) - cos(thetaz_r)*sin(thetax_r)*cos(thetay_r);
        Rwn_31 = -1*cos(thetax_r)*sin(thetay_r);
        Rwn_32 = sin(thetax_r);
        Rwn_33 = cos(thetax_r)*cos(thetay_r);

        *tau1 = fric_coeff_1*u_fric_1 - (JT_11*Rwn_11 + JT_12*Rwn_21 + JT_13*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_11*Rwn_12 + JT_12*Rwn_22 + JT_13*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_11*Rwn_13 + JT_12*Rwn_23 + JT_13*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau2 = fric_coeff_2*u_fric_2 - (JT_21*Rwn_11 + JT_22*Rwn_21 + JT_23*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_21*Rwn_12 + JT_22*Rwn_22 + JT_23*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_21*Rwn_13 + JT_22*Rwn_23 + JT_23*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau3 = fric_coeff_3*u_fric_3 - (JT_31*Rwn_11 + JT_32*Rwn_21 + JT_33*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_31*Rwn_12 + JT_32*Rwn_22 + JT_33*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_31*Rwn_13 + JT_32*Rwn_23 + JT_33*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
    }

    if(controllerdesired == 4)
    {
        Kpxn = 600;
        Kpyn = 50;
        Kpzn = 600;

        Kdxn = 100;
        Kdyn = 5;
        Kdzn = 100;

        thetaz_r = -0.261;

        Rwn_11 = cos(thetaz_r)*cos(thetay_r) - sin(thetaz_r)*sin(thetax_r)*sin(thetay_r);
        Rwn_12 = -1*sin(thetaz_r)*cos(thetax_r);
        Rwn_13 = cos(thetaz_r)*sin(thetay_r) + sin(thetaz_r)*sin(thetax_r)*cos(thetay_r);
        Rwn_21 = sin(thetaz_r)*cos(thetay_r) + cos(thetaz_r)*sin(thetax_r)*sin(thetay_r);
        Rwn_22 = cos(thetaz_r)*cos(thetax_r);
        Rwn_23 = sin(thetaz_r)*sin(thetay_r) - cos(thetaz_r)*sin(thetax_r)*cos(thetay_r);
        Rwn_31 = -1*cos(thetax_r)*sin(thetay_r);
        Rwn_32 = sin(thetax_r);
        Rwn_33 = cos(thetax_r)*cos(thetay_r);

        *tau1 = fric_coeff_1*u_fric_1 - (JT_11*Rwn_11 + JT_12*Rwn_21 + JT_13*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_11*Rwn_12 + JT_12*Rwn_22 + JT_13*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_11*Rwn_13 + JT_12*Rwn_23 + JT_13*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau2 = fric_coeff_2*u_fric_2 - (JT_21*Rwn_11 + JT_22*Rwn_21 + JT_23*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_21*Rwn_12 + JT_22*Rwn_22 + JT_23*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_21*Rwn_13 + JT_22*Rwn_23 + JT_23*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
        *tau3 = fric_coeff_3*u_fric_3 - (JT_31*Rwn_11 + JT_32*Rwn_21 + JT_33*Rwn_31)*(Kdxn*Rwn_11*(vx_endeffector - vxtask_d) + Kdxn*Rwn_21*(vy_endeffector - vytask_d) + Kdxn*Rwn_31*(vz_endeffector - vztask_d) + Kpxn*Rwn_11*(x_endeffector - xtask_d) + Kpxn*Rwn_21*(y_endeffector - ytask_d) + Kpxn*Rwn_31*(z_endeffector - ztask_d)) - (JT_31*Rwn_12 + JT_32*Rwn_22 + JT_33*Rwn_32)*(Kdyn*Rwn_12*(vx_endeffector - vxtask_d) + Kdyn*Rwn_22*(vy_endeffector - vytask_d) + Kdyn*Rwn_32*(vz_endeffector - vztask_d) + Kpyn*Rwn_12*(x_endeffector - xtask_d) + Kpyn*Rwn_22*(y_endeffector - ytask_d) + Kpyn*Rwn_32*(z_endeffector - ztask_d)) - (JT_31*Rwn_13 + JT_32*Rwn_23 + JT_33*Rwn_33)*(Kdzn*Rwn_13*(vx_endeffector - vxtask_d) + Kdzn*Rwn_23*(vy_endeffector - vytask_d) + Kdzn*Rwn_33*(vz_endeffector - vztask_d) + Kpzn*Rwn_13*(x_endeffector - xtask_d) + Kpzn*Rwn_23*(y_endeffector - ytask_d) + Kpzn*Rwn_33*(z_endeffector - ztask_d));
    }


    if(controllerdesired == 5)
    {
        //        Kpz_task = 40; //Kpz_task
        //        Kdz_task = 0;
        Kpx_task = 800;
        Kpy_task = 800;
        Kpz_task = 800;
        Kdx_task = 40;
        Kdy_task = 60;
        Kdz_task = 60;

        *tau1 = fric_coeff_1*u_fric_1 +  JT_11*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_12*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_13*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_13*Fzcmd/Kt;
        *tau2 = fric_coeff_2*u_fric_2 +  JT_21*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_22*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_23*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_23*Fzcmd/Kt;
        *tau3 = fric_coeff_3*u_fric_3 +  JT_31*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_32*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_33*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_33*Fzcmd/Kt;
    }

    if(controllerdesired == 7)
    {
        //        Kpz_task = 40; //Kpz_task
        //        Kdz_task = 0;
        Kpx_task = 400;
        Kpy_task = 400;
        Kpz_task = 800;
        Kdx_task = 0.5;
        Kdy_task = 0.5;
        Kdz_task = 0.5;

        *tau1 = fric_coeff_1*u_fric_1 +  JT_11*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_12*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_13*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_13*Fzcmd/Kt;
        *tau2 = fric_coeff_2*u_fric_2 +  JT_21*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_22*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_23*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_23*Fzcmd/Kt;
        *tau3 = fric_coeff_3*u_fric_3 +  JT_31*(Kpx_task*(xtask_d - x_endeffector) + Kdx_task*(vxtask_d - vx_endeffector)) + JT_32*(Kpy_task*(ytask_d - y_endeffector) + Kdy_task*(vytask_d - vy_endeffector)) + JT_33*(Kpz_task*(ztask_d - z_endeffector) + Kdz_task*(vztask_d - vz_endeffector)) + JT_33*Fzcmd/Kt;
        if(*tau1 < -3)
        {
            *tau1 = -3;
            Ik_theta1 = Ik_theta1_1;
        }

        if(*tau1 > 3)
        {
            *tau1 = 5;
            Ik_theta1 = Ik_theta1_1;
        }

        if(*tau2 < -3)
        {
            *tau2 = -3;
            Ik_theta2 = Ik_theta2_1;

        }

        if(*tau2 > 3)
        {
            *tau2 = 3;
            Ik_theta2 = Ik_theta2_1;
        }

        if(*tau3 < -3)
        {
            *tau3 = -5;
            Ik_theta3 = Ik_theta3_1;
        }

        if(*tau3 > 3)
        {
            *tau3 = 3;
            Ik_theta3 = Ik_theta3_1;
        }
    }
    //pks11
    //torque saturation code:
    // if tourque is greater than 5; saturate to 5
    //if torque is less than -5, saturate to -5
    //this is for all 3 joints

    if(*tau1 < -5)
    {
        *tau1 = -5;
    }

    if(*tau1 > 5)
    {
        *tau1 = 5;
    }

    if(*tau2 < -5)
    {
        *tau2 = -5;
    }

    if(*tau2 > 5)
    {
        *tau2 = 5;
    }

    if(*tau3 < -5)
    {
        *tau3 = -5;
    }

    if(*tau3 > 5)
    {
        *tau3 = 5;
    }



    //pks11
    //FUN TRAJECTORY
    // TRAJECTORY CHOOSE : Planner Circle with radius 0.1m, starting position : (0.4,01,0.35)
    // Center : (0.3,0.1,0.35)
    // FREQUENCY : 1/5 = 0.2 Hz (takes 5 seconds to finish 1 circle)


    //    //Aplying Trajecory of my own //pks11
    //    x_endeffector = 0.3 + 0.1*cos(2*PI*dt*mycount/T);
    //    y_endeffector = 0.1 + 0.1*sin(2*PI*dt*mycount/T);
    //    z_endeffector = 0.35;

    //pks11_ calculating thetas from inverese kinematics from the geomteric approach
    theta1_calc = atan(y_endeffector/x_endeffector);
    theta3_calc = acos((pow((L- z_endeffector),2) + pow(x_endeffector,2) + pow(y_endeffector,2) - 2*pow(L,2))/(2*pow(L,2)));
    //  theta2_calc = acos(sqrt((pow(x_endeffector,2) + pow(y_endeffector,2))/((pow(x_endeffector,2) + pow(y_endeffector,2) + pow((L- z_endeffector),2))))) - (theta3_calc/2);
    theta2_calc = atan2((L-z_endeffector),sqrt((pow(x_endeffector,2) + pow(y_endeffector,2)))) - (theta3_calc/2);

    //    // Measured the relationship between theta angles and joint motors angles.
    //    //Relationship : theta2DH = theta2m - pi/2;
    //    // theta3DH = -theta2m + theta3m + pi/2;
    //    theta1_motor_calc = theta1_calc;
    //    theta2_motor_calc = theta2_calc + PI/2;
    //    theta3_motor_calc = theta3_calc + theta2motor - PI/2;
    //
    //    theta1_desired = theta1_motor_calc;
    //    theta2_desired = theta2_motor_calc;
    //    theta3_desired = theta3_motor_calc;
    //pks11 - sending reference signals and motor angles
    //this is the interface between simulink and codecomposer!
    Simulink_PlotVar1 = xtask_d;
    Simulink_PlotVar2 = x_endeffector;
    Simulink_PlotVar3 = ytask_d;
    Simulink_PlotVar4 = y_endeffector;


    //state update
    e_theta1_1 = e_theta1;
    e_theta2_1 = e_theta2;
    e_theta3_1 = e_theta3;
    Ik_theta1_1 = Ik_theta1;
    Ik_theta2_1 = Ik_theta2;
    Ik_theta3_1 = Ik_theta3;
    vx_taskd_k_2 = vx_taskd_k_1;
    vx_taskd_k_1 = vxtask_d;
    vy_taskd_k_2 = vy_taskd_k_1;
    vy_taskd_k_1 = vytask_d;
    vz_taskd_k_2 = vz_taskd_k_1;
    vz_taskd_k_1 = vztask_d;



    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        //serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        //  serial_printf(&SerialA, "%.2f %.2f,%.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,theta3); //printing theta3 in radians as well! pks11
        serial_printf(&SerialA, "theamotor : %.2f,%.2f,%.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI); // pks11// this is motors angle measured
        serial_printf(&SerialA, "position : %.5f,%.5f,%.5f \n\r",x_endeffector,y_endeffector,z_endeffector); //pks11 // position calculated from the forward Kinematics // D-H Table
        serial_printf(&SerialA, "thetaDHcalc : %.2f, %.2f, %.2f \n\r",theta1_calc*180/PI,theta2_calc*180/PI,theta3_calc*180/PI); // pks11 // DH theta calculated from the geometric approach of the inverese kinematics
        serial_printf(&SerialA, "thetamotorcalc :%.2f,%.2f,%.2f \n\r",theta1_motor_calc*180/PI,theta2_motor_calc*180/PI,theta3_motor_calc*180/PI); // pks11/ Joint motors calculated from the calculted inverese kinemtaics of DH Theta

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

