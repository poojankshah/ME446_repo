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

//pks11
float  x_endeffector = 0;
float  y_endeffector = 0;
float  z_endeffector = 0;

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

//error
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
float Kp2_PID = 200; //old 40
float Kp3_PID = 150; // old 50

float Kd1_PID = 2;
float Kd2_PID = 7; // old 2
float Kd3_PID = 7;

float Ki1_PID = 10; // old 5
float Ki2_PID = 75; // old 5
float Ki3_PID = 75; // old 7.5

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

    //pks11 defining the input
    //    if ((mycount%1000)== 0) {
    //        theta1_desired = PI/6;
    //        theta2_desired = PI/6;
    //        theta3_desired = PI/6;
    //        if((mycount%2000) == 0){
    //            theta1_desired = 0;
    //            theta2_desired = 0;
    //            theta3_desired = 0;
    //        }
    //    }



    //Trajectory Generation
    //Rising theta = 0 to theta = 0.5 in 0_1 second
    a0_1 = 0;
    a1_1 = 0;
    a2_1 = 1.5;
    a3_1 = -1;

    //declining, theta - 0.5 to theta = 0 in 1-2 second
    a0_2 = -2;
    a1_2 = 6;
    a2_2 = -4.5;
    a3_2 = 1;
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
    //pks11 Implmenting the theta dot filters
    //theta1dot :: Omega1
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    //order matters here. Why??
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    //theta2dot : Omega2
    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    //order matters here. Why??
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    //theta2dot : Omega2
    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    //order matters here. Why??
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    /// pks11 Tracking errors calculation
    //trackingerror_theta1

    e_theta1 = theta1_desired - theta1motor;

    //trackingerror_theta2
    e_theta2 = theta2_desired - theta2motor;

    //trackingerror_theta3
    e_theta3 = theta3_desired - theta3motor;

    //pid controller Adaptive gains!
    if(fabs(e_theta1) < 0.08)
    {

        //*tau1 = Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 - Kd1_PID*(Omega1);  //pks11 : Older PID
        //*tau1 = J1*theta1dotdot_desired + Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 + Kd1_PID*(theta1dot_desired - Omega1);
        *tau1 = Kp1_PID*(theta1_desired - theta1motor) + Ki1_PID*Ik_theta1 - Kd1_PID*(Omega1);
        Ik_theta1 = Ik_theta1_1 + (e_theta1 + e_theta1_1)*0.001;
    }
    else
    {
        Ik_theta1 = 0;
        //*tau1 =J1*theta1dotdot_desired + Kp1_PD*(theta1_desired - theta1motor) + Kd1_PD*(theta1dot_desired - Omega1);
        *tau1 = Kp1_PID*(theta1_desired - theta1motor) - Kd1_PID*(Omega1);
    }

    //theta2 control
    if(fabs(e_theta2) < 0.08)
    {

        //*tau2 = J2*theta2dotdot_desired + Kp2_PID*(theta2_desired - theta2motor) + Ki2_PID*Ik_theta2 + Kd2_PID*(theta2dot_desired - Omega2);
        *tau2 = Kp2_PID*(theta2_desired - theta2motor) + Ki2_PID*Ik_theta2 - Kd2_PID*(Omega2);
        Ik_theta2 = Ik_theta2_1 + (e_theta2 + e_theta2_1)*0.001;
    }
    else
    {
        Ik_theta2 = 0;
        //*tau2 = J2*theta2dotdot_desired + Kp2_PD*(theta2_desired - theta2motor) + Kd2_PD*(theta2dot_desired - Omega2);
        *tau2 = Kp2_PID*(theta2_desired - theta2motor) - Kd2_PID*(Omega2);
    }

    //theta3 control
    if(fabs(e_theta3) < 0.08)
    {
        //*tau3 = J3*theta3dotdot_desired + Kp3_PID*(theta3_desired - theta3motor) + Ki3_PID*Ik_theta3 + Kd3_PID*(theta3dot_desired - Omega3);
        *tau3 = Kp3_PID*(theta3_desired - theta3motor) + Ki3_PID*Ik_theta3 - Kd3_PID*(Omega3);
        Ik_theta3 = Ik_theta3_1 + (e_theta3 + e_theta3_1)*0.001;
    }

    else
    {
        Ik_theta3 = 0;
        //*tau3 = J3*theta3dotdot_desired + Kp3_PD*(theta3_desired - theta3motor) + Kd3_PD*(theta3dot_desired - Omega3);
        *tau3 = Kp3_PID*(theta3_desired - theta3motor) - Kd3_PID*(Omega3);
    }


    //// PD
    //pks11 calculating the taus // pd controller
    //    *tau1 = Kp1*(theta1_desired - theta1motor) - Kd1*(Omega1);
    //    *tau2 = Kp2*(theta2_desired - theta2motor) - Kd2*(Omega2);
    //    *tau3 = Kp3*(theta3_desired - theta3motor) - Kd3*(Omega3);


    //Saturation block
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




    //pks11 adding a calculation of h03 with motor angles
    //Steps Folllowed : Calculated forward kinemtics from the DH table and provide the coordinates of endeffector which is [0;0;0] in frame 3
    // Measured the relationship between theta angles and joint motors angles.
    //Relationship : theta2DH = theta2m - pi/2;
    // theta3DH = -theta2m + theta3m + pi/2;
    x_endeffector = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y_endeffector = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z_endeffector = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

    //Aplying Trajecory of my own //pks11
    if(mycount<5000)
    {
        x_endeffector = 0.45 + 0.1*cos(2*PI*dt*mycount/T);
        y_endeffector = 0 + 0.1*sin(2*PI*dt*mycount/T);
    }
    z_endeffector = 0.15;

    //pks11_ calculating thetas from inverese kinematics from the geomteric approach
    theta1_calc = atan(y_endeffector/x_endeffector);
    theta3_calc = acos((pow((L- z_endeffector),2) + pow(x_endeffector,2) + pow(y_endeffector,2) - 2*pow(L,2))/(2*pow(L,2)));
    //  theta2_calc = acos(sqrt((pow(x_endeffector,2) + pow(y_endeffector,2))/((pow(x_endeffector,2) + pow(y_endeffector,2) + pow((L- z_endeffector),2))))) - (theta3_calc/2);
    theta2_calc = atan2((L-z_endeffector),sqrt((pow(x_endeffector,2) + pow(y_endeffector,2)))) - (theta3_calc/2);

    // Measured the relationship between theta angles and joint motors angles.
    //Relationship : theta2DH = theta2m - pi/2;
    // theta3DH = -theta2m + theta3m + pi/2;
    theta1_motor_calc = theta1_calc;
    theta2_motor_calc = theta2_calc + PI/2;
    theta3_motor_calc = theta3_calc + theta2motor - PI/2;

    theta1_desired = theta1_motor_calc;
    theta2_desired = theta2_motor_calc;
    theta3_desired = theta3_motor_calc;
    //pks11 - sending reference signals and motor angles
    Simulink_PlotVar1 = e_theta1;
    Simulink_PlotVar2 = e_theta2;
    Simulink_PlotVar3 = e_theta3;
    Simulink_PlotVar4 = theta3_desired;


    //state update
    e_theta1_1 = e_theta1;
    e_theta2_1 = e_theta2;
    e_theta3_1 = e_theta3;
    Ik_theta1_1 = Ik_theta1;
    Ik_theta2_1 = Ik_theta2;
    Ik_theta3_1 = Ik_theta3;


    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        //serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        //  serial_printf(&SerialA, "%.2f %.2f,%.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,theta3); //printing theta3 in radians as well! pks11
        serial_printf(&SerialA, "theamotor : %.2f,%.2f,%.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI); // pks11// this is motors angle measured
        serial_printf(&SerialA, "position : %.2f,%.2f,%.2f \n\r",x_endeffector,y_endeffector,z_endeffector); //pks11 // position calculated from the forward Kinematics // D-H Table
        serial_printf(&SerialA, "thetaDHcalc : %.2f, %.2f, %.2f \n\r",theta1_calc*180/PI,theta2_calc*180/PI,theta3_calc*180/PI); // pks11 // DH theta calculated from the geometric approach of the inverese kinematics
        serial_printf(&SerialA, "thetamotorcalc :%.2f,%.2f,%.2f \n\r",theta1_motor_calc*180/PI,theta2_motor_calc*180/PI,theta3_motor_calc*180/PI); // pks11/ Joint motors calculated from the calculted inverese kinemtaics of DH Theta

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

