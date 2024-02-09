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


float L = 0.254; //pks11 // Link parameter in meters

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

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    //pks11 adding a variable
    theta3 = theta3motor;

    //pks11 adding a calculation of h03 with motor angles
    //Steps Folllowed : Calculated forward kinemtics from the DH table and provide the coordinates of endeffector which is [0;0;0] in frame 3
    // Measured the relationship between theta angles and joint motors angles.
    //Relationship : theta2DH = theta2m - pi/2;
    // theta3DH = -theta2m + theta3m + pi/2;
    x_endeffector = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y_endeffector = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z_endeffector = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

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

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        //serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
//  serial_printf(&SerialA, "%.2f %.2f,%.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,theta3); //printing theta3 in radians as well! pks11
        serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI); // pks11// this is motors angle measured
        serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r",x_endeffector,y_endeffector,z_endeffector); //pks11 // position calculated from the forward Kinematics // D-H Table
        serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r",theta1_calc*180/PI,theta2_calc*180/PI,theta3_calc*180/PI); // pks11 // DH theta calculated from the geometric approach of the inverese kinematics
        serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r",theta1_motor_calc*180/PI,theta2_motor_calc*180/PI,theta3_motor_calc*180/PI); // pks11/ Joint motors calculated from the calculted inverese kinemtaics of DH Theta

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

